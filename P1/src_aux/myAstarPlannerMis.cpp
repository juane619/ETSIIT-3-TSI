#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>

//MIO
const uint MAX_INT = 0xffffffff;
const int CALC_H = 0; //DISTINTAS HEURISTICAS: {0:euclidean, 1:chebyshev}
const int MAX_EXPLORADOS = 30000;
const float TBREAK = 0;
const double E = 0.15;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {

	MyastarPlanner::MyastarPlanner()
	: costmap_ros_(NULL), initialized_(false) {
	}

	MyastarPlanner::MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
	: costmap_ros_(NULL), initialized_(false) {
		initialize(name, costmap_ros);
	}

	void MyastarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
		if (!initialized_) {
			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();

			ros::NodeHandle private_nh("~/" + name);

			//vamos a asumir estos parámetros, que no es necesario enviar desde el launch.
			private_nh.param("step_size", step_size_, costmap_->getResolution());
			private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

			//el plan se va a publicar en el topic "planTotal"
			plan_pub_ = private_nh.advertise<nav_msgs::Path>("planTotal", 1);

			//los puntos del espacio de búsqueda se visualizan en "visualization_marker"
			marker_Open_publisher = private_nh.advertise<visualization_msgs::Marker>("open_list", 1000);
			marker_Closed_publisher = private_nh.advertise<visualization_msgs::Marker>("closed_list", 1000);
			marker_Goals_publisher = private_nh.advertise<visualization_msgs::Marker>("goals_markers", 1000);

			initialized_ = true;
		} else
			ROS_WARN("This planner has already been initialized... doing nothing");
	}

	//we need to take the footprint of the robot into account when we calculate cost to obstacles

	double MyastarPlanner::footprintCost(double x_i, double y_i, double theta_i) {
		if (!initialized_) {
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return -1.0;
		}

		//ROS_INFO("Viendo footprint(cost de huella): ");
		std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

		//if we have no footprint... do nothing
		if (footprint.size() < 3)
			return -1.0;

		double footprint_cost = MyCostmapModel(*costmap_).footprintCost(x_i, y_i, theta_i, footprint);
		//ROS_INFO("cost: %f", footprint_cost);
		//cin.get();

		return footprint_cost;
	}

	bool MyastarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
		//***********************************************************
		// Inicio de gestion de ROS
		//***********************************************************
		if (!initialized_) {
			ROS_ERROR("The astar planner has not been initialized, please call initialize() to use the planner");
			return false;
		}

		ROS_DEBUG("MyastarPlanner: Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

		/**************************************************************************/
		/*************** GESTION VISUALIZACION PUNTOS DE ABIERTOS Y CERRADOS********/
		/****************************************************************************/

		//visualization_msgs::Marker points;// definida en la clase como markers_OpenList
		inicializaMarkersPoints(markers_OpenList, "openList", 0, 0.0f, 1.0f, 0.0f);
		inicializaMarkersPoints(markers_ClosedList, "closedList", 1, 1.0f, 0.0f, 0.0f);
		inicializaMarkersLine_List(markers_Goals, "goals", 2, 0.0f, 0.0f, 1.0f);

		limpiaMarkers(marker_Open_publisher, markers_ClosedList);
		limpiaMarkers(marker_Closed_publisher, markers_OpenList);

		/**************************************************************************/
		/*************** FIN GESTION VISUALIZACION PUNTOS DE ABIERTOS Y CERRADOS***/
		/**************************************************************************/

		//obtenemos el costmap global  que esta publicado por move_base.
		costmap_ = costmap_ros_->getCostmap();

		//Obligamos a que el marco de coordenadas del goal enviado y del costmap sea el mismo.
		//esto es importante para evitar errores de transformaciones de coordenadas.
		if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
			ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
					costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
			return false;
		}

		tf::Stamped<tf::Pose> goal_tf;
		tf::Stamped<tf::Pose> start_tf;

		poseStampedMsgToTF(goal, goal_tf);
		poseStampedMsgToTF(start, start_tf);

		//obtenemos la orientacion start y goal en start_yaw y goal_yaw.
		double useless_pitch, useless_roll, goal_yaw, start_yaw;
		start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
		goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

		/**************************************************************************/
		/*************** HASTA AQUI GESTIoN DE ROS *********************************/
		/****************************************************************************/

		//Reset data structures
		plan.clear();
		openList.clear();
		came_from.clear();
		cost_so_far.clear();

		uint explorados = 0; //llevamos cuenta de los nodos explorados para parar el algoritmo
		uint currentIndex; //nodo actual

		//pasamos el goal y start a nuestros tipo de datos
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		uint mgoal_x, mgoal_y;
		costmap_->worldToMap(goal_x, goal_y, mgoal_x, mgoal_y);
		uint index_goal = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);

		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
		uint mstart_x, mstart_y;
		costmap_->worldToMap(start_x, start_y, mstart_x, mstart_y);
		uint index_start = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);

		//START A* ALGORITHM

		//insertamos la casilla inicial en abiertos con f = 0 ya que es el unico y por lo tanto no hace falta calcularla
		MyastarPlanner::openList.put(index_start, 0);
		
		//parametros para dynamic weighting
		double N = calculateHCost(index_start, index_goal, CALC_H);
		double w; //pesos

		//visualizamos start.
		visualizaCelda(marker_Open_publisher, markers_OpenList, index_start);

		//medimos tiempo
		ros::Time t_ini = ros::Time::now();

		while (explorados++ < MAX_EXPLORADOS && !MyastarPlanner::openList.empty()) //while the open list is not empty continuie the search
		{
			//set padre y g del inicial
			came_from[index_start] = index_start;
			cost_so_far[index_start] = 0;

			//escoger mejor nodo de ABIERTOS (mejor f) y lo quitamos de ABIERTOS
			currentIndex = openList.get();

			//No utilizamos una closedList. Nos basta con las estructuras utilizadas

			visualizaCelda(marker_Closed_publisher, markers_ClosedList, currentIndex);

			// if the currentCell is the goalCell: success: path found
			if (currentIndex == index_goal) {
				//el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
				//vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
				ROS_INFO("Se han explorado %u nodos.", explorados);
				//ros::Duration(10).sleep();

				//convertimos goal a poseStamped nueva
				geometry_msgs::PoseStamped pose;
				pose.header.stamp = ros::Time::now();
				pose.header.frame_id = goal.header.frame_id; //debe tener el mismo frame que el de la entrada
				pose.pose.position.x = goal_x;
				pose.pose.position.y = goal_y;
				pose.pose.position.z = 0.0;
				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				//lo aniadimos al plan
				plan.push_back(pose);

				uint currentParent = came_from[currentIndex];

				while (currentParent != index_start) //e.d. mientras no lleguemos al nodo start
				{
					//creamos una PoseStamped con la informacion de currentCouple.index

					//primero hay que convertir el current parent a world coordinates
					uint mpose_x, mpose_y;
					double wpose_x, wpose_y;

					costmap_->indexToCells(currentParent, mpose_x, mpose_y);
					costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

					//después creamos la pose
					geometry_msgs::PoseStamped pose;
					pose.header.stamp = ros::Time::now();
					pose.header.frame_id = goal.header.frame_id; //debe tener el mismo frame que el de la entrada
					pose.pose.position.x = wpose_x;
					pose.pose.position.y = wpose_y;
					pose.pose.position.z = 0.0;
					pose.pose.orientation.x = 0.0;
					pose.pose.orientation.y = 0.0;
					pose.pose.orientation.z = 0.0;
					pose.pose.orientation.w = 1.0;

					//insertamos la pose en el plan
					plan.push_back(pose);

					//hacemos que currentParent sea el parent de currentParent
					currentParent = came_from[currentParent];
				}

				std::reverse(plan.begin(), plan.end());
				ros::Time t_fin = ros::Time::now();

				ROS_INFO("Plan generado. Nodos explorados: %u. Tiempo: %f. Nodos plan: %zu.", explorados, (t_fin - t_ini).toSec(), plan.size());

				return true;
			}

			//search the neighbors of the current Cell ignoring those that impassable (footprint)
			vector<uint> neighborCells = findFreeNeighborCell(currentIndex);
			set<uint> neighborsNotInOpenList; //utilizada solo para mostrar en Rviz la expansion

			for (auto next : neighborCells) {
				double new_cost = cost_so_far[currentIndex] + getMoveCost(currentIndex, next); //g vecino

				//aniadimos si no estan en abiertos, o, si esta ya en abiertos, y su coste g es mayor que el coste g pasando por currentIndex
				if (cost_so_far.find(next) == cost_so_far.end() || new_cost < cost_so_far[next]) {
					neighborsNotInOpenList.insert(next); //Rviz
					cost_so_far[next] = new_cost; //guardamos la g del nuevo nodo añadido (vecino)

					//dynamic weighting
					double hcost = calculateHCost(next, index_goal, CALC_H);
					//					if (hcost <= N)
					//						w = 1 - hcost / N;
					//					else
					//						w = 0;
					w = 1 + E * hcost / N;

					//hcost = (1 + E * w) * hcost;
					w=1;
					//hcost = w * hcost;
					//ROS_INFO("Peso(add): %f", w);

					double priority = new_cost + w*hcost; //calculate f with weights
					openList.put(next, priority); //add neigh with f cost(priority)
					came_from[next] = currentIndex; //set next parent to currentIndex
				}
			}

			explorados++;

			//PINTO ABIERTOS
			//Anyadir neighborCells a points. pushback()

			visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
			visualizaCelda(marker_Closed_publisher, markers_ClosedList, currentIndex);
		}

		if (explorados >= MAX_EXPLORADOS || openList.empty()) // if the openList is empty: then failure to find a path
		{

			ros::Time t_fin2 = ros::Time::now();
			ROS_INFO("Failure to find a path!. Nodos explorados: %u. Tiempo: %f.", explorados, (t_fin2 - t_ini).toSec());
			return false;
			// exit(1);
		}

	};

	//calculamos H como la distancia euclídea hasta el goal.
	//uint mode especifica la heurística usada (0:euclidean)

	double MyastarPlanner::calculateHCost(uint start, uint goal, uint mode) {
		uint mstart_x, mstart_y, mgoal_x, mgoal_y;
		double wstart_x, wstart_y, wgoal_x, wgoal_y;

		//trasformamos el indice de celdas a coordenadas del mundo.
		//ver http://docs.ros.org/indigo/api/costmap_2d/html/classcostmap__2d_1_1Costmap2D.html

		costmap_->indexToCells(start, mstart_x, mstart_y);
		costmap_->mapToWorld(mstart_x, mstart_y, wstart_x, wstart_y);
		costmap_->indexToCells(goal, mgoal_x, mgoal_y);
		costmap_->mapToWorld(mgoal_x, mgoal_y, wgoal_x, wgoal_y);

		double distance, dx, dy, D, D2;

		switch (mode) {
			case 0: //EUCLIDEAN
				dx = std::abs(wstart_x - wgoal_x);
				dy = std::abs(wstart_y - wgoal_y);

				distance = sqrt(dx * dx + dy * dy);
				break;
			case 1://EUCLIDEAN, SQUARED, NOT RECOMENDED(SCALE)
				dx = std::abs(wstart_x - wgoal_x);
				dy = std::abs(wstart_y - wgoal_y);

				distance = D * (dx * dx + dy * dy);
				break;
			case 2://OCTILE DISTANCE
				dx = std::abs(wstart_x - wgoal_x);
				dy = std::abs(wstart_y - wgoal_y);
				D = 1;
				D2 = sqrt(2);

				distance = D * (dx + dy)+(D2 - 2 * D) * std::min(dx, dy);
				break;
			case 3://CHEBYSHEV DISTANCE
				dx = std::abs(wstart_x - wgoal_x);
				dy = std::abs(wstart_y - wgoal_y);
				D = 1;
				D2 = 1;

				distance = D * (dx + dy)+(D2 - 2 * D) * std::min(dx, dy);
				break;

		}

		return distance;
	}

	/*******************************************************************************
	 * Function Name: findFreeNeighborCell
	 * Inputs: the row and columun of the current Cell
	 * Output: a vector of free neighbor cells of the current cell
	 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
	 * Check Status: Checked by Anis, Imen and Sahar
	 *********************************************************************************/
	vector<uint> MyastarPlanner::findFreeNeighborCell(uint parent) {
		uint mx, my;
		double wx, wy;
		costmap_->indexToCells(parent, mx, my);
		//ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx, my);

		vector <uint> freeNeighborCells;

		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {
				if (x != 0 || y != 0) {
					//check whether the index is valid
					if ((mx + x >= 0)&&(mx + x < costmap_->getSizeInCellsX())&&(my + y >= 0)&&(my + y < costmap_->getSizeInCellsY())) {
						//world coordinates of neighs
						costmap_->mapToWorld((uint) mx + x, (uint) my + y, wx, wy);

						unsigned char cost = footprintCost(wx, wy, 1.0);
						//ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
						if (cost < 255) {
							uint index = costmap_->getIndex(mx + x, my + y);
							freeNeighborCells.push_back(index);
						}
					}
				}
			}
		}
		return freeNeighborCells;
	}


	//obtiene el gCost como la distancia euclídea(mode=0)

	double MyastarPlanner::getMoveCost(uint here, uint there) {
		//calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
		return calculateHCost(here, there);
	}

	//END A*

	/********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

	void MyastarPlanner::inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
		marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
		marker.header.stamp = ros::Time::now();
		marker.ns = ns;

		marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
		marker.pose.orientation.w = 0.0;

		marker.id = id;
		marker.type = visualization_msgs::Marker::POINTS;

		// POINTS markers use x and y scale for width/height respectively
		marker.scale.x = costmap_->getResolution();
		marker.scale.y = costmap_->getResolution();

		// Points are green
		marker.color.g = g;
		marker.color.r = r;
		marker.color.b = b;
		marker.color.a = 1.0;
	}

	void MyastarPlanner::inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b) {
		marker.header.frame_id = costmap_ros_->getGlobalFrameID().c_str();
		marker.header.stamp = ros::Time::now();
		marker.ns = ns;

		marker.action = visualization_msgs::Marker::ADD; //la otra es DELETE
		marker.pose.orientation.w = 0.0;
		marker.pose.position.x = 0.0;
		marker.pose.position.y = 0.0;

		marker.id = id;
		marker.type = visualization_msgs::Marker::SPHERE;

		//Line lists also have some special handling for scale: only scale.x is used and it controls the width of the line segments.
		marker.scale.x = marker.scale.y = 0.5;
		// marker.scale.y = costmap_->getResolution();

		// Points are green
		marker.color.g = g;
		marker.color.r = r;
		marker.color.b = b;
		marker.color.a = 1.0;
	}

	void MyastarPlanner::visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y) {
		//PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
		geometry_msgs::Point p;
		p.x = x;
		p.y = y;
		p.z = 0; //¿?

		marker.points.push_back(p); //anyado el punto inicial
		where.publish(marker); //lo publico
		//points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
	}

	void MyastarPlanner::visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z) {
		//PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution

		marker.pose.position.x = x;
		marker.pose.position.y = y;
		where.publish(marker); //lo publico
		//points.points.pop_back(); //quito el punto de la lista de puntos, lo borro con DELETE cuando lo saque de abiertos.
	}

	void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, uint index) {
		uint mpose_x, mpose_y;
		double wpose_x, wpose_y;
		costmap_->indexToCells(index, mpose_x, mpose_y);
		costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
		visualizaCoords(where, marker, wpose_x, wpose_y);

	}

	void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, set<uint> &lista) {
		for (set<uint>::iterator i = lista.begin(); i != lista.end(); ++i) {
			uint mpose_x, mpose_y;
			double wpose_x, wpose_y;
			costmap_->indexToCells(*i, mpose_x, mpose_y);
			costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
			//PINTO: cpstart.x, cpstart.y, scale == costmap_->getResolution
			geometry_msgs::Point p;
			p.x = wpose_x;
			p.y = wpose_y;
			p.z = 0; //¿?

			marker.points.push_back(p);
		}
		where.publish(marker);
		//quitar neighborCells de points .popback
	}

	void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker) {
		if (!marker.points.empty()) {
			marker.action = visualization_msgs::Marker::DELETE;
			where.publish(marker);
			marker.action = visualization_msgs::Marker::ADD;
		}
		marker.points.clear();
	}
}
