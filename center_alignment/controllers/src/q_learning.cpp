#include <controllers/q_learning.h>

void positionCallback(const Vector3Stamped& msg){
    position = msg.vector;
}

void attitudeCallback(const Vector3Stamped& msg){
    attitude = msg.vector;
}

// Constructor
q_learning::q_learning(int argc, char** argv){
    ros::init(argc, argv, "q_learning");
    ros::NodeHandle node_handle;

    ros::NodeHandle nh("~");

    nh.getParam("initial_x", position.x);
    nh.getParam("initial_y", position.y);
    nh.getParam("initial_z", position.z);

    cout << "*** " << position.x << ", " << position.y << ", " << position.z << endl;

    char path[50];
    snprintf(path, sizeof(char) * 50, "/home/andriy/Results/trajectory_%i_%i_%i.txt", (int)position.x, (int)position.y, (int)position.z);
    results.open(path);

    std::string file;
    nh.getParam("policy_file_name", file);

    position_subscriber = node_handle.subscribe("/vtol/position", 1, positionCallback);
    attitude_subscriber = node_handle.subscribe("/vtol/attitude", 1, attitudeCallback);

    velocity_publisher = node_handle.advertise<Vector3Stamped>("/vtol/input/velocity", 1);

    fstream fin("../catkin_ws/src/controllers/policy/q_learning.txt", ios::in);
    for(int i = 0; i < MAP_HEIGHT * MAP_LENGTH * MAP_WIDTH; ++i){
        fin.get(policy[(int)(i / (MAP_WIDTH * MAP_LENGTH))][(int)(i / MAP_WIDTH) % MAP_LENGTH][i % MAP_WIDTH]);
        if(policy[(int)(i / (MAP_WIDTH * MAP_LENGTH))][(int)(i / MAP_WIDTH) % MAP_LENGTH][i % MAP_WIDTH] == '\n')
            --i;
    }

    /*for(int k = 0; k < MAP_HEIGHT; ++k){
        for(int j = 0; j < MAP_LENGTH; ++j){
            for(int i = 0; i < MAP_WIDTH; ++i)
                cout << policy[k][j][i];
            cout << endl;
        }
        cout << endl;
    }*/
}

// Destructor
q_learning::~q_learning(){
    ros::shutdown();
    exit(0);
}

void q_learning::setVelocity(double vx, double vy, double vz){
    if(velocity.x != vx || velocity.y != vy || velocity.y != vy){
        velocity_prev.x = velocity.x;
        velocity_prev.y = velocity.y;
        velocity_prev.z = velocity.z;
        velocity.x = vx;
        velocity.y = vy;
        velocity.z = vz;
    }

}

Vector3 q_learning::computeVelocity(){
    Vector3 v;
    v.x = velocity.x + velocity_prev.x;
    v.y = velocity.y + velocity_prev.y;
    v.z = velocity.z + velocity_prev.z;
    return v;
}

void q_learning::saveData(){
    results << position.x << ", " << position.y << ", " << position.z << endl;
}

void q_learning::run(){            
    ros::Rate rate(100);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();

        //cout << "position: [" << position.x << ", " << position.y << ", " << position.z << "] -> [" << (int)floor(position.x + MAP_WIDTH / 2) << ", " << (int)floor(-position.y + MAP_LENGTH / 2) << ", " << (int)ceil(position.z + 0.5) << "]";
        char action = policy[(int)ceil(position.z)][(int)floor(-position.y + MAP_LENGTH / 2)][(int)floor(position.x + MAP_WIDTH / 2)];
        //cout << " -> " << action;

        switch(action){
        case 'F':
            setVelocity(0, MAX_VELOCITY, 0);
            break;
        case 'B':
            setVelocity(0, -MAX_VELOCITY, 0);
            break;
        case 'L':
            setVelocity(-MAX_VELOCITY, 0, 0);
            break;
        case 'R':
            setVelocity(MAX_VELOCITY, 0, 0);
            break;
        case 'U':
            setVelocity(0, 0, MAX_VELOCITY);
            break;
        case 'D':
            setVelocity(0, 0, -MAX_VELOCITY);
            break;
        case 'G':
            setVelocity(0, 0, 0);
            exit(0);
        case '#':
            setVelocity(0, 0, 0);
            exit(0);
        default:
            setVelocity(0, 0, 0);
            //cout << "ERROR! Unknown action!!! (" << action << ")" << endl;
        }

        Vector3Stamped message_velocity;
        message_velocity.header.stamp = ros::Time::now();
        message_velocity.vector = computeVelocity();
        velocity_publisher.publish(message_velocity);

        //cout << " -> velocity: [" << message_velocity.vector.x << ", " << message_velocity.vector.y << ", " << message_velocity.vector.z << "]" << endl;

        saveData();
    }
}

int main(int argc, char** argv){
    cout << "Q-Learning is running..." << endl;

    q_learning* ql = new q_learning(argc, argv);

    ql->run();

}
