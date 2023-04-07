/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/

#include "planner.h"

double distance_angles(double* angles1, double* angles2, int numofDOFs)
{
    double dist = 0;
	for(int i=0; i<numofDOFs; i++)
	{
		dist = dist + (angles1[i]-angles2[i]) * (angles1[i]-angles2[i]);
	}
	return sqrt(dist);
}

int newConfig(double* q, double* q_near, double* q_new, int numofDOFs, double* map, int x_size, int y_size) 
{
    // move by EPSILON towards q from q_near and return q_new

    double dist = 0;
    int success = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(dist < fabs(q_near[j] - q[j]))
            dist = fabs(q_near[j] - q[j]);
    }
    int numofsamples = (int)(dist/(PI/20));

    double* tmp_angles = (double*)malloc(numofDOFs*sizeof(double));

    for (i = 1; i < numofsamples; i++)
    {
    	for(j = 0; j<numofDOFs; j++)
    	{
    		tmp_angles[j] = q_near[j] + ((double)(i)/(numofsamples-1))*(q[j] - q_near[j]);
    	}
    	if(IsValidArmConfiguration(tmp_angles, numofDOFs, map, x_size, y_size) && 
    		distance_angles(tmp_angles, q_near, numofDOFs) < EPSILON)
    	{
    		memcpy(q_new, tmp_angles, numofDOFs*sizeof(double));
            success = 1;
    	}
    	else
    	{break;}
    }

    free(tmp_angles);
    return success;

}

int isAtGoal(double* angles, double* goal_angles, int numofDOFs)
{
    int reached = 0;
    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(angles[j] - goal_angles[j]))
            distance = fabs(angles[j] - goal_angles[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        reached = 1;
    }
    return reached;
}

// Extend tree towards the sample node (RRT and RRT-Connect)
int extendTree(Tree* tree, double* q, int numofDOFs, double* map, int x_size, int y_size)
{
    int status = TRAPPED;
    int q_near_id = tree->nearestNeighbour(q);
    double* q_near = tree->getNode(q_near_id);
    double* q_new = (double*)malloc(numofDOFs*sizeof(double));
    if(newConfig(q, q_near, q_new, numofDOFs, map, x_size, y_size)) 
    {
        int q_new_id = tree->addNode(q_new);
        tree->addEdge(q_new_id, q_near_id);

		if(isAtGoal(q_new, q, numofDOFs))
			status = REACHED;
		else
			status = ADVANCED;
    }
    return status;
}

// Extend tree towards the sample node (RRT star)
int extendTreeRRTStar(Tree* tree, double* q, int numofDOFs, double* map, int x_size, int y_size)
{
    int status = TRAPPED;
	double cost_new = 0.0;
	double dist = 0.0;
	double radius = EPSILON;
    int q_near_id = tree->nearestNeighbour(q);
    double* q_near = tree->getNode(q_near_id);
    double* q_new = (double*)malloc(numofDOFs*sizeof(double));
    if(newConfig(q, q_near, q_new, numofDOFs, map, x_size, y_size)) 
    {
        int q_new_id = tree->addNode(q_new);
        tree->addEdge(q_new_id, q_near_id);
		tree->costs[q_new_id] = tree->distBetweenNodes(q_new, q_near);

		for(int i=0; i<tree->nodes.size(); i++)
		{
			dist = tree->distBetweenNodes(q_new, tree->nodes[i]);
			if(dist < radius)
			{
				if(cost_new + dist < tree->costs[i])
				{
					tree->costs[i] = cost_new + dist;
					tree->addEdge(i, q_new_id);
				}
			}
		}

		if(isAtGoal(q_new, q, numofDOFs))
			status = REACHED;
		else
			status = ADVANCED;
    }
    return status;
}

// Generate Random Config
double* randomConfig(int numofDOFs, double* map, int x_size, int y_size) {
    double* sample_node_rad = new double[numofDOFs];
    // Check if config is valid
    int isvalid = 0;
    while(!isvalid) {
        for (int i = 0; i < numofDOFs; i++) {
            int random_deg = rand() % 360;
            sample_node_rad[i] = (double)random_deg / 180 * PI;
        }
        if(IsValidArmConfiguration(sample_node_rad, numofDOFs, map, x_size, y_size)) {
            isvalid = 1;
        }
    }
    return sample_node_rad;
}

void swap(Tree *tree1, Tree *tree2)
{
	Tree temp = *tree1;
	*tree1 = *tree2;
	*tree2 = temp;
}

int connect(Tree* tree_b, double* q_new, int numofDOFs, double* map, int x_size, int y_size)
{
	int status = TRAPPED;
	do{
		status = extendTree(tree_b, q_new, numofDOFs, map, x_size, y_size);
	}
	while(status == ADVANCED);
	return status;
}

bool aStarInClosedList(int idx, unordered_set<int> closed_list)
{
    if (closed_list.find(idx) == closed_list.end())
        return false;
    else
        return true;
}

// PRM Planner
static void PRMplanner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	Graph graph(numofDOFs, armstart_anglesV_rad, armgoal_anglesV_rad);

	// build roadmap
	int i = 0;
	int N = 1500*5;

	int goal_idx = graph.addNode(armgoal_anglesV_rad);
	int num_edges = 0;

	double R = EPSILON;
	int k = 15; // max number of connections to neighbours

	clock_t tick = clock();
	while(i < N)
	{
		i++;
		int n_connected = 0;

		double* alpha_i = randomConfig(numofDOFs, map, x_size, y_size);
		int alpha_i_idx = graph.addNode(alpha_i);

		for(int i=0; i<graph.nodes.size()-1; i++)
		{
			if(distance_angles(graph.nodes[i], alpha_i, numofDOFs) < R)
			{
				double* q_new = (double*)malloc(numofDOFs*sizeof(double));
				newConfig(graph.nodes[i], alpha_i, q_new, numofDOFs, map, x_size, y_size);
				if(isAtGoal(q_new, alpha_i, numofDOFs))
				{
					graph.addEdge(i, alpha_i_idx);
					n_connected++;
					num_edges++;
					if(n_connected >= k)
						break;
				}
			}
		}
	}
	clock_t tock = clock();
	printf("Goal connections: %d\n", graph.edges[1].size());
	printf("Start connections: %d\n", graph.edges[0].size());
	printf("Number of edges: %d\n", num_edges);
	printf("Time taken to build roadmap: %f seconds\n", (float)(tock-tick)/CLOCKS_PER_SEC);

	// find path using A*

	struct node
    {
        int parent = -1; // idx of previous(parent) node
        int g = std::numeric_limits<int>::max(); // set g value to inf initially
    };

	priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> open_list;
    unordered_set<int> closed_list;
    unordered_map<int, node> node_info;

	bool goal_found = false;

	node_info[0].g = 0;
	open_list.push(make_pair(0, 0));

	while(!open_list.empty() && !goal_found)
	{
		pair<int, int> current_node = open_list.top(); 
		open_list.pop();
		if(aStarInClosedList(current_node.second, closed_list))
			continue;
		closed_list.insert(current_node.second);

		// iterate over successors
		for(int i=0; i<graph.edges[current_node.second].size(); i++)
		{
			int neighbour_idx = graph.edges[current_node.second][i];
			if(aStarInClosedList(neighbour_idx, closed_list))
				continue;
			int cost_to_neighbour = (int) distance_angles(graph.nodes[current_node.second], graph.nodes[neighbour_idx], numofDOFs);
			if(node_info[neighbour_idx].g > node_info[current_node.second].g + cost_to_neighbour)
			{
				node_info[neighbour_idx].parent = current_node.second;
				node_info[neighbour_idx].g = node_info[current_node.second].g + cost_to_neighbour;
				int h = (int) distance_angles(graph.nodes[neighbour_idx], graph.nodes[1], numofDOFs);
				open_list.push(make_pair(node_info[neighbour_idx].g + h, neighbour_idx));
			}
		}

		if(aStarInClosedList(1, closed_list))
			goal_found = true;
	}

	if(goal_found)
	{
		vector<int> path;
		int current_node = 1;
		while(current_node != 0)
		{
			path.insert(path.begin(),current_node);
			current_node = node_info[current_node].parent;
		}
		path.insert(path.begin(), 0);

		*planlength = path.size();
		*plan = (double**)malloc(path.size()*sizeof(double*));
		for(int i=0; i<path.size(); i++)
		{
			(*plan)[i] = (double*)malloc(numofDOFs*sizeof(double));
			for(int j=0; j<numofDOFs; j++)
				(*plan)[i][j] = graph.nodes[path[i]][j];
		}
	}
	else
	{
		printf("No path found\n");
	}

}

// RRT Star Planner
static void RRTStarplanner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// Initialize tree structure with start node
	Tree tree(numofDOFs, armstart_anglesV_rad);

	// Number of samples
	int K = 1000000;
	int k = 0;

	// Goal bias
	double goal_bias = 0.1;

	// target found flag
	bool target_found = 0;

	while(!target_found && k < K)
	{
		k++;

		double* q_rand = (double*)malloc(numofDOFs*sizeof(double));
		// Sample a random node with goal bias
		if((double)rand() / RAND_MAX < goal_bias)
		{
			// Sample goal node
			// cout << "Sampling goal node" << endl;
			q_rand = armgoal_anglesV_rad;
		}
		else
		{
			// Sample random node
			// cout << "Sampling random node" << endl;
			q_rand = randomConfig(numofDOFs, map, x_size, y_size);
		}
		
		// Extend the tree towards the sample node
		if(extendTreeRRTStar(&tree, q_rand, numofDOFs, map, x_size, y_size) == TRAPPED)
		{
			continue;
		}

		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);

		// Check if the new node is close to the goal
		if (isAtGoal(q_new, armgoal_anglesV_rad, numofDOFs))
		{
			target_found = 1;
			cout << "Target found" << endl;
			cout << "Number of samples: " << k << endl;
			if (!equalDoubleArrays(q_new, armgoal_anglesV_rad, numofDOFs))
			{
				// we are very near to goal but not quite there
				// so add the goal to the tree
				int node_id = tree.addNode(armgoal_anglesV_rad);
				tree.addEdge(node_id,q_new_id);
			}
		}
	}

	// If target is found, construct and return the plan
	if(target_found)
	{
		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);
		vector<int> path;
		int next_id = q_new_id;
		while (next_id != 0) {
			path.insert(path.begin(), next_id);
			next_id = tree.getParentID(next_id);
		}
		path.insert(path.begin(), 0);
		*planlength = path.size();
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for(int i=0; i<path.size(); i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree.getNode(path[i]), numofDOFs*sizeof(double));
		}
	}
	else
	{
		printf("Target not found\n");
	}	
}

// RRT Connect Planner
static void RRTConnectplanner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
	int total_path_length = 0;

	// Initialize tree structure with start node
	Tree tree_a(numofDOFs, armstart_anglesV_rad);
	Tree tree_b(numofDOFs, armgoal_anglesV_rad);

	// Number of samples
	int K = 1000000;
	int k = 0;

	// target found flag
	bool connected = 0;

	while(!connected && k < K)
	{
		k++;

		double* q_rand = (double*)malloc(numofDOFs*sizeof(double));
		// Sample random node
		// cout << "Sampling random node" << endl;
		q_rand = randomConfig(numofDOFs, map, x_size, y_size);
		
		// Extend the tree towards the sample node
		if(extendTree(&tree_a, q_rand, numofDOFs, map, x_size, y_size) != TRAPPED)
		{
			int q_new_id = tree_a.getNewNodeID();
			double* q_new = tree_a.getNode(q_new_id);
			if(connect(&tree_b, q_new, numofDOFs, map, x_size, y_size) == REACHED)
			{
				connected = 1;
				cout << "Connected" << endl;
				cout << "Number of samples: " << k << endl;
				break;
			}
		}
		swap(&tree_a, &tree_b);
	}

	// If both trees connect, construct and return the plan
	if(connected)
	{

		// Make tree_a as start tree and tree_b as goal tree
		double *q = tree_b.getNode(0);
		if(!isAtGoal(q, armgoal_anglesV_rad, numofDOFs))
			swap(&tree_a, &tree_b);


		// Construct the plan

		// Extract path from goal tree (tree_b)
		vector<int> path_b;
		int q_new_id = tree_b.getNewNodeID();
		double* q_new = tree_b.getNode(q_new_id);
		int next_id = q_new_id;
		while (next_id != 0) {
			path_b.insert(path_b.end(), next_id);
			next_id = tree_b.getParentID(next_id);
		}
		path_b.insert(path_b.end(), 0);

		// Extract path from start tree (tree_a)
		vector<int> path_a;
		q_new_id = tree_a.getNewNodeID();
		q_new = tree_a.getNode(q_new_id);
		next_id = q_new_id - 1;
		while (next_id != 0) {
			path_a.insert(path_a.begin(), next_id);
			next_id = tree_a.getParentID(next_id);
		}
		path_a.insert(path_a.begin(), 0);

		total_path_length = path_a.size() + path_b.size();
		*planlength = total_path_length;
		*plan = (double**) malloc(total_path_length*sizeof(double*));
		for(int i=0; i<path_a.size(); i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree_a.getNode(path_a[i]), numofDOFs*sizeof(double));
		}
		for(int i=path_a.size(); i<total_path_length; i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree_b.getNode(path_b[i-path_a.size()]), numofDOFs*sizeof(double));
		}
	}
	else
	{
		printf("Trees did not connect\n");
	}

	int countNumInvalid = 0;
    for (int i = 0; i < total_path_length; i++)
	{
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
			++countNumInvalid;
        }
    }
	printf("Collided at %d instances across the path\n", countNumInvalid);


}

// RRT Planner
static void RRTplanner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;

	// Initialize tree structure with start node
	Tree tree(numofDOFs, armstart_anglesV_rad);

	// Number of samples
	int K = 1000000;
	int k = 0;

	// Goal bias
	double goal_bias = 0.05;

	// target found flag
	bool target_found = 0;

	while(!target_found && k < K)
	{
		k++;

		double* q_rand = (double*)malloc(numofDOFs*sizeof(double));
		// Sample a random node with goal bias
		if((double)rand() / RAND_MAX < goal_bias)
		{
			// Sample goal node
			// cout << "Sampling goal node" << endl;
			q_rand = armgoal_anglesV_rad;
		}
		else
		{
			// Sample random node
			// cout << "Sampling random node" << endl;
			q_rand = randomConfig(numofDOFs, map, x_size, y_size);
		}
		
		// Extend the tree towards the sample node
		if(extendTree(&tree, q_rand, numofDOFs, map, x_size, y_size) == TRAPPED)
		{
			continue;
		}

		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);

		// Check if the new node is close to the goal
		if (isAtGoal(q_new, armgoal_anglesV_rad, numofDOFs))
		{
			target_found = 1;
			cout << "Target found" << endl;
			cout << "Number of samples: " << k << endl;
			if (!equalDoubleArrays(q_new, armgoal_anglesV_rad, numofDOFs))
			{
				// we are very near to goal but not quite there
				// so add the goal to the tree
				int node_id = tree.addNode(armgoal_anglesV_rad);
				tree.addEdge(node_id,q_new_id);
			}
		}
	}

	// If target is found, construct and return the plan
	if(target_found)
	{
		int q_new_id = tree.getNewNodeID();
		double* q_new = tree.getNode(q_new_id);
		vector<int> path;
		int next_id = q_new_id;
		while (next_id != 0) {
			path.insert(path.begin(), next_id);
			next_id = tree.getParentID(next_id);
		}
		path.insert(path.begin(), 0);
		*planlength = path.size();
		*plan = (double**) malloc(path.size()*sizeof(double*));
		for(int i=0; i<path.size(); i++)
		{
			(*plan)[i] = (double*) malloc(numofDOFs*sizeof(double));
			memcpy((*plan)[i], tree.getNode(path[i]), numofDOFs*sizeof(double));
		}
	}
	else
	{
		printf("Target not found\n");
	}	

}


static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("The arm is already at the goal\n");
        return;
    }
	int countNumInvalid = 0;
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
			++countNumInvalid;
        }
    }
	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
    *planlength = numofsamples;
    
    return;
}


/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	// seed
	srand( (unsigned)time( NULL ) );
	// srand(1);

	double** plan = NULL;
	int planlength = 0;

	switch(whichPlanner) {
		case RRT:
			RRTplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case RRTCONNECT:
			RRTConnectplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case RRTSTAR:
			RRTStarplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		case PRM:
			PRMplanner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
			break;
		default:
			throw runtime_error("Invalid planner number!\n");
	}

	if (!equalDoubleArrays(plan[0], startPos, numOfDOFs))
	{
		throw std::runtime_error("Start position not matching");
	}
	if (!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs))
	{
		throw std::runtime_error("Goal position not matching");
	}

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
