/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Eitan Marder-Eppstein, Sachin Chitta
 *********************************************************************/
#include "../include/my_astar_planner/myAstarPlanner.h"
#include <pluginlib/class_list_macros.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

// para debugging
#include <sstream>
#include <string>
#include <vector>
#include <list>

//MIO
#define it_open list<coupleOfCells>::iterator
const uint MAX_INT = 0xffffffff;
const int CALC_H = 2; //DISTINTAS HEURISTICAS: {0:euclidean, 1:chebyshev}
const int MAX_EXPLORADOS = 30000;

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(myastar_planner::MyastarPlanner, nav_core::BaseGlobalPlanner)

namespace myastar_planner {
	list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID);
	bool isContains(list<coupleOfCells> & list1, int cellID);

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
			//world_model_ = new base_local_planner::CostmapModel(*costmap_);


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

		ROS_INFO("Viendo footprint(cost de huella): ");
		std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();

		//if we have no footprint... do nothing
		if (footprint.size() < 3)
			return -1.0;

		double footprint_cost = MyCostmapModel(*costmap_).footprintCost(x_i, y_i, theta_i, footprint);
		ROS_INFO("cost: %f", footprint_cost);
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


		plan.clear();
		closedList.clear();
		openList.clear();

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

		//pasamos el goal y start a estructura coupleOfCells
		coupleOfCells cpstart, cpgoal;
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		unsigned int mgoal_x, mgoal_y;
		costmap_->worldToMap(goal_x, goal_y, mgoal_x, mgoal_y);
		cpgoal.index = MyastarPlanner::costmap_->getIndex(mgoal_x, mgoal_y);
		cpgoal.parent = 0;
		cpgoal.gCost = 0;
		cpgoal.hCost = 0;
		cpgoal.fCost = 0;

		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
		unsigned int mstart_x, mstart_y;
		costmap_->worldToMap(start_x, start_y, mstart_x, mstart_y);

		ROS_INFO("Coste huella antes de planificar");
		footprintCost(start_x, start_y, 1.0);
		int e;
		//cin >> e;

		cpstart.index = MyastarPlanner::costmap_->getIndex(mstart_x, mstart_y);
		cpstart.parent = cpstart.index;
		cpstart.gCost = 0;
		cpstart.hCost = 0;
		cpstart.fCost = 0;

		//insertamos la casilla inicial en abiertos
		MyastarPlanner::openList.push_back(cpstart);
		//ROS_INFO("Inserto en Abiertos: %d", cpstart.index);
		//ROS_INFO("Index del goal: %d", cpgoal.index);


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
		/*************** FIN GESTION VISUALIZACION PUNTOS DE ABIERTOS Y CERRADOS********/
		/****************************************************************************/

		//visualizamos start.
		visualizaCelda(marker_Open_publisher, markers_OpenList, cpstart.index);

		unsigned int explorados = 0;
		unsigned int currentIndex = cpstart.index;

		//medimos tiempo
		ros::Time t_ini = ros::Time::now();

		while (explorados++ < MAX_EXPLORADOS && !MyastarPlanner::openList.empty()) //while the open list is not empty continuie the search
		{
			//escoger mejor nodo de abiertos

			//ordenamos openList
			if (openList.size() > 1) {
				openList.sort(compareFCost);
			}
			//escogemos el de mejor f
			coupleOfCells bestCOfCells = openList.front();

			//lo quitamos de abiertos
			openList.pop_front();
			currentIndex = bestCOfCells.index;

			//y lo insertamos en cerrados
			MyastarPlanner::closedList.push_back(bestCOfCells);

			visualizaCelda(marker_Closed_publisher, markers_ClosedList, currentIndex);

			// if the currentCell is the goalCell: success: path found
			if (currentIndex == cpgoal.index) {
				//el plan lo construimos partiendo del goal, del parent del goal y saltando en cerrados "de parent en parent"
				//vamos insertando al final los waypoints (los nodos de cerrados), por tanto, cuando finaliza el bucle hay que darle la vuelta al plan
				//ROS_INFO("Se han explorado %u nodos y cerrados tiene %u nodos", explorados, (unsigned int) closedList.size());
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

				coupleOfCells currentCouple = cpgoal;
				unsigned int currentParent = bestCOfCells.parent;
				//ROS_INFO("Inserta en Plan GOAL: %f, %f PADRE: %u", pose.pose.position.x, pose.pose.position.y, currentParent);
				//ros::Duration(1).sleep();

				while (currentParent != cpstart.index) //e.d. mientras no lleguemos al nodo start
				{
					//encontramos la posicion de currentParent en cerrados
					list<coupleOfCells>::iterator it = getPositionInList(closedList, currentParent);

					//hacemos esa posicion que sea el currentCouple
					coupleOfCells currentCouple;
					currentCouple.index = currentParent;
					currentCouple.parent = (*it).parent;
					currentCouple.gCost = (*it).gCost;
					currentCouple.hCost = (*it).hCost;
					currentCouple.fCost = (*it).fCost;


					//creamos una PoseStamped con la informacion de currentCouple.index

					//primero hay que convertir el currentCouple.index a world coordinates
					unsigned int mpose_x, mpose_y;
					double wpose_x, wpose_y;

					costmap_->indexToCells((*it).index, mpose_x, mpose_y);
					costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);

					//ROS_INFO("Las coordenadas de El PADRE de %u son (%u, %u) -> (%f, %f). Y su PADRE es %u.", currentParent, mpose_x, mpose_y, wpose_x, wpose_y, (*it).parent);
					//ros::Duration(1).sleep();

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
					//ROS_INFO("Inserta en Plan: %f, %f", pose.pose.position.x, pose.pose.position.y);
					//hacemos que currentParent sea el parent de currentCouple
					currentParent = (*it).parent;
				}

				std::reverse(plan.begin(), plan.end());
				ros::Time t_fin = ros::Time::now();

				ROS_INFO("Plan generado. Nodos explorados: %u. Tiempo: %f. Nodos plan: %zu.", explorados, (t_fin - t_ini).toSec(), plan.size());

				//lo publica en el topic "planTotal"
				//publishPlan(plan);
				return true;
			}

			//search the neighbors of the current Cell
			vector<unsigned int> neighborCells = findFreeNeighborCell(currentIndex);

			//ROS_INFO("Ha encontrado %u vecinos", (unsigned int) neighborCells.size());

			//neighbors that exist in the closedList are ignored
			vector<unsigned int> neighborNotInClosedList;
			for (uint i = 0; i < neighborCells.size(); i++) {
				if (!isContains(closedList, neighborCells[i])) {
					neighborNotInClosedList.push_back(neighborCells[i]);
				}
			}

			//ROS_INFO("Ha encontrado %u vecinos que no estan en cerrados", (unsigned int) neighborNotInClosedList.size());

			vector<unsigned int> neighborsNotInOpenList;
			vector<unsigned int> neighborsInOpenList;

			//search the neighbors that already exist in the open List
			for (uint i = 0; i < neighborNotInClosedList.size(); i++) {
				uint neigh_index = neighborNotInClosedList[i];
				if (!isContains(openList, neigh_index))
					neighborsNotInOpenList.push_back(neigh_index);
				else
					neighborsInOpenList.push_back(neigh_index);
			}

			addNeighborCellsToOpenList(openList, neighborsNotInOpenList, currentIndex, cpstart.gCost, cpgoal.index, cpstart.index);
			explorados++;

			//PINTO ABIERTOS
			//Anyadir neighborCells a points. pushback()

			visualizaLista(marker_Open_publisher, markers_OpenList, neighborsNotInOpenList);
			visualizaCelda(marker_Closed_publisher, markers_ClosedList, bestCOfCells.index);

			//Para los nodos que ya estan en abiertos, comprobar en cerrados su coste y actualizarlo si fuera necesario
			//propagarInformacion(openList, neighborNotInClosedList, currentIndex, bestCOfCells.gCost);
		}

		if (explorados >= MAX_EXPLORADOS || openList.empty()) // if the openList is empty: then failure to find a path
		{
			ROS_INFO("Failure to find a path !");
			return false;
			// exit(1);
		}

	};


	//calculamos H como la distancia euclídea hasta el goal

	double MyastarPlanner::calculateHCost(unsigned int start, unsigned int goal, uint mode) {
		unsigned int mstart_x, mstart_y, mgoal_x, mgoal_y;
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

				return distance;
		}
	}


	//comparamos F para dos nodos.

	bool MyastarPlanner::compareFCost(coupleOfCells const &c1, coupleOfCells const &c2) {
		return c1.fCost < c2.fCost;
	}

	/*******************************************************************************/
	//Function Name: getPositnionInList
	//Inputs:the cellID, the list
	//Output: index of the cell in the list
	//Description: it is used to search the index of a cell in a list

	/*********************************************************************************/
	list<coupleOfCells>::iterator getPositionInList(list<coupleOfCells> & list1, unsigned int cellID) {
		for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++) {
			if (it->index == cellID)
				return it;

		}
	}

	/*propagarInformacion
	 *	
	 */

	void MyastarPlanner::propagarInformacion(list<coupleOfCells>& openList, const vector<unsigned int> &neighs, uint &id_best, double &cost_best) {

		for (int i = 0; i < neighs.size(); i++) {
			coupleOfCells aux;
			unsigned int id_actual = neighs[i];

			it_open salida = getPositionInList(openList, id_actual);

			if (salida != openList.end()) {
				double previus_cost = salida->gCost;

				double coste = cost_best + getMoveCost(id_best, id_actual);
				if (coste < previus_cost) {
					salida->parent = id_best;
					salida->gCost = coste;
					salida->fCost = .3 * aux.gCost + 0.7 * aux.hCost;
				}
			}
		}
	}

	/*******************************************************************************
	 * Function Name: findFreeNeighborCell
	 * Inputs: the row and columun of the current Cell
	 * Output: a vector of free neighbor cells of the current cell
	 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
	 * Check Status: Checked by Anis, Imen and Sahar
	 *********************************************************************************/
	vector<unsigned int> MyastarPlanner::findFreeNeighborCell(unsigned int parent) {
		unsigned int mx, my;
		double wx, wy;
		costmap_->indexToCells(parent, mx, my);
		//ROS_INFO("Viendo vecinos de index: %u, Map coords: (%u,%u)", CellID, mx, my);

		vector <unsigned int> freeNeighborCells;

		for (int x = -1; x <= 1; x++) {
			for (int y = -1; y <= 1; y++) {
				if (x != 0 || y != 0) {
					//check whether the index is valid
					if ((mx + x >= 0)&&(mx + x < costmap_->getSizeInCellsX())&&(my + y >= 0)&&(my + y < costmap_->getSizeInCellsY())) {
						//world coordinates of neighs
						costmap_->mapToWorld((unsigned int) mx + x, (unsigned int) my + y, wx, wy);
						//neigh valido(cercano a obstaculo o no)
						//bool valid = (footprintCost(wx, wy, 1) != MAX_INT);

						//ROS_INFO("Comprobando casilla con Map coords(%u,%u), World coords (%f,%f)", mx+x, my+y ,wx,wy);
						unsigned char cost = costmap_->getCost(mx + x, my + y);

						if (cost < 15) {
							//ROS_INFO("Coste de la casilla vecina: %d", cost);
							unsigned int index = costmap_->getIndex(mx + x, my + y);
							freeNeighborCells.push_back(index);
						}
					}
				}
			}
		}
		return freeNeighborCells;
	}


	/*******************************************************************************/
	//Function Name: isContains
	//Inputs: the list, the cellID
	//Output: true or false
	//Description: it is used to check if a cell exists in the open list or in the closed list

	/*********************************************************************************/
	bool isContains(list<coupleOfCells> & list1, int cellID) {
		for (list<coupleOfCells>::iterator it = list1.begin(); it != list1.end(); it++) {
			if (it->index == cellID)
				return true;
		}
		return false;
	}

	double MyastarPlanner::getMoveCost(unsigned int here, unsigned int there) {
		//calculo el coste de moverme entre celdas adyacentes como la distancia euclídea.
		return calculateHCost(here, there);

	}

	/*******************************************************************************/
	//Function Name: addNeighborCellsToOpenList
	//Inputs: the open list, the neighbors Cells and the parent Cell
	//Output:
	//Description: it is used to add the neighbor Cells to the open list

	/*********************************************************************************/
	void MyastarPlanner::addNeighborCellsToOpenList(list<coupleOfCells> & OPL, vector <unsigned int> neighborCells, unsigned int parent, float gCostParent, unsigned int goalCell, uint startCell) //,float tBreak)
	{
		uint neigh_index;
		uint mx, my;
		double wx, wy;
		double w; //pesos

		for (uint i = 0; i < neighborCells.size(); i++) {
			neigh_index = neighborCells[i];
			coupleOfCells CP;
			costmap_->indexToCells(neigh_index, mx, my); //para posterior calculo de seguridad
			costmap_->mapToWorld(mx, my, wx, wy);

			CP.index = neigh_index; //insert the neighbor cell
			CP.parent = parent; //insert the parent cell
			//calculamos g incluyendo el coste del robot(footprint)
			CP.gCost = gCostParent + getMoveCost(parent, neigh_index);

			//con los pesos
			//w = calculateHCost(neigh_index, goalCell) / calculateHCost(startCell, goalCell);
			CP.hCost = calculateHCost(neigh_index, goalCell, CALC_H);
			CP.fCost = CP.gCost + CP.hCost;

			OPL.push_back(CP);
		}
	}

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

	void MyastarPlanner::visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int index) {
		unsigned int mpose_x, mpose_y;
		double wpose_x, wpose_y;
		costmap_->indexToCells(index, mpose_x, mpose_y);
		costmap_->mapToWorld(mpose_x, mpose_y, wpose_x, wpose_y);
		visualizaCoords(where, marker, wpose_x, wpose_y);

	}

	void MyastarPlanner::visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, vector<unsigned int> lista) {
		for (vector<unsigned int>::iterator i = lista.begin(); i != lista.end(); ++i) {
			unsigned int mpose_x, mpose_y;
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

	void MyastarPlanner::limpiaMarkers(ros::Publisher where, visualization_msgs::Marker & marker) {
		if (!marker.points.empty()) {
			marker.action = visualization_msgs::Marker::DELETE;
			where.publish(marker);
			marker.action = visualization_msgs::Marker::ADD;
		}
		marker.points.clear();


	}
}
