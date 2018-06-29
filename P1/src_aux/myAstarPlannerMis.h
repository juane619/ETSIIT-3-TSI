/*A* COMO LISTAS COMENTADO Y CON LAS NUEVAS ESTRUCTURAS IMPLEMENTADAS*/

#ifndef MYASTAR_PLANNER_H_
#define MYASTAR_PLANNER_H_

//includes para integrarse en ROS
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h> //??
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

//para pintar puntos
#include <visualization_msgs/Marker.h>

//includes especificos para hacer referencia a la implementacion del algoritmo astar.

#include <vector>
#include <unordered_map>
#include <set>
#include <iostream>

namespace myastar_planner {
    /**
     * @class myastar_planner
     * @brief Provides an a-star simple global planner that will compute a valid goal point for the local planner by using classical astar implementation.
     */

    using namespace std;

    /*Change data structures A*/

        /**
     * @struct PriorityQueue
     * @brief A struct that represents a priority queur that can assign priority and compare nodes(pair)
     * Node have an int ID and double f(priority) 
     */
    struct PriorityQueue {
        typedef std::pair<double, int> PQElement;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement> > elements;

        inline bool empty() const {
            return elements.empty();
        }

        inline void put(uint item, double priority) {
            elements.emplace(priority, item);
        }

        uint get() {
            uint best_item = elements.top().second;
            elements.pop();
            return best_item;
        }

        void print() {
            std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement> > aux;
            while (!elements.empty()) {
                PQElement auxel = elements.top();
                aux.emplace(auxel);
                elements.pop();
                cout << "item: " << auxel.second << ", f: " << auxel.first << endl;
            }
            while (!aux.empty()) {
                PQElement auxel = aux.top();
                elements.emplace(auxel);
                aux.pop();
            }
        }

        void clear() {
            elements = std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement> >();
        }
    };

    /*END CHANGES DATA STRUCTURES*/

    class MyCostmapModel : public base_local_planner::CostmapModel {
    public:

        MyCostmapModel(costmap_2d::Costmap2D& c) : CostmapModel(c) {
        }
    };

    class MyastarPlanner : public nav_core::BaseGlobalPlanner { //implementa la interfaz que provee nav_core::BaseGlobalPlanner
    public:
        /**
         * @brief  Constructor for the MyastarPlanner
         */
        MyastarPlanner();
        /**
         * @brief  Constructor for the MyastarPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        MyastarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief  Initialization function for MyastarPlanner
         * @param  name The name of this planner
         * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
         */
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /**
         * @brief Given a goal pose in the world, compute a plan
         * @param start The start pose
         * @param goal The goal pose
         * @param plan The plan... filled by the planner
         * @return True if a valid plan was found, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

    private:
        //necesarios para manejar el costmap y el footprint.
        costmap_2d::Costmap2DROS* costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D* costmap_;

        //para publicar el plan
        ros::Publisher plan_pub_;

        //necesarios para manejar las listas de abiertos y cerrados de astar.
        
        //!< the open list: it contains all the expanded cells (current cells) with ID and fCost
        PriorityQueue openList; 

        //!< management closed list (simulated), gCosts and parents
        std::unordered_map<uint, uint> came_from;
        std::unordered_map<uint, double> cost_so_far;

        /**
         * @brief  Checks the legality of the robot footprint at a position and orientation using the world model
         * @param x_i The x position of the robot
         * @param y_i The y position of the robot
         * @param theta_i The orientation of the robot
         * @return
         */
        double footprintCost(double x_i, double y_i, double theta_i);
        /**
         * @brief  Calcula la estimacion del costo de ir desde una casilla (definida por su indice) hasta otra (definida por su indice)
         * @param start El indice de la casilla inicial
         * @param end El indice de la casilla final
         * @return
         */
        double calculateHCost(unsigned int start, unsigned int goal, uint mode = 0);

        //devuelve celdas adyacentes a CellID que estan libres
        vector<unsigned int> findFreeNeighborCell(unsigned int parent);

        //devuelve el costo de moverse desde una casilla "here" hasta otra casilla "there"
        double getMoveCost(unsigned int here, unsigned int there);

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

        /********VISUALIZAR ESPACIO DE BUSQUEDA *************************/

        //para publicar puntos de abiertos y cerrados
        ros::Publisher marker_Open_publisher;
        ros::Publisher marker_Closed_publisher;
        
        //para publicar las marcas de los goals
        ros::Publisher marker_Goals_publisher;
        
        //necesario para guardar la lista de puntos de abiertos y cerrados a visualizar como markers en rviz.
        visualization_msgs::Marker markers_OpenList;
        visualization_msgs::Marker markers_ClosedList;
        
        //necesario para guardar las line_list que marcan los puntos objetivo recomendados.
        visualization_msgs::Marker markers_Goals;

        void inicializaMarkersPoints(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
        void inicializaMarkersLine_List(visualization_msgs::Marker &marker, string ns, int id, float r, float g, float b);
        void visualizaCoords(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y);
        void visualizaCoordsLineUp(ros::Publisher where, visualization_msgs::Marker &marker, double x, double y, double z);
        void visualizaCelda(ros::Publisher where, visualization_msgs::Marker &marker, unsigned int cell);


        void visualizaLista(ros::Publisher where, visualization_msgs::Marker &marker, set<uint> &lista);
        void limpiaMarkers(ros::Publisher where, visualization_msgs::Marker &marker);

        /************FIN VISUALIZAR ESPACIO DE BUSQUEDA *******************************/

        bool initialized_;
    };
};
#endif
