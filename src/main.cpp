#include "camera_manager.hpp"


int main(int argc, char * argv[])
{

    ros::init(argc, argv, "camera_manager");

    ROS_INFO("Realsense ROS Interface");
    
    ros::NodeHandle nh_local("~"); // Node handle should be created in the main thread!
    auto camera_ns_list = CameraParam::loadCameras(nh_local);

    std::cout << camera_ns_list.size() << " Cameras to be loaded..." << std::endl;

    std::vector<std::unique_ptr<CameraManager>> camera_list;

    

    for (auto& camera_ns : camera_ns_list)
    {
        std::unique_ptr<CameraManager> camera(new CameraManager(camera_ns));

        if (camera->isInitialised())
            camera_list.push_back(std::move(camera));
        else
        {
            camera.reset();
        }
    }

    // start Pipes
    // Start image process in different threads
    std::vector<std::thread> process_list;

    for (auto& camera : camera_list)
    {
        camera->startPipe();
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    }

    for (auto& camera : camera_list)
        process_list.push_back(std::thread(&CameraManager::processFrame, camera.get()));
        

    // ros::AsyncSpinner spinner(2);
	// spinner.start();

    ROS_INFO("Start Spinning");

    if (process_list.size() != 0)
        ros::spin();
    else
    {
        std::cerr << "No cameras are successfully loaded..." << std::endl;
    }
    
    std::cout << "Cleaning up..." << std::endl;

    for (auto& process : process_list)
    {
        process.join();
    }

    for (auto& camera : camera_list)
        camera.reset(); // delete the pointer

    // spinner.stop();

    std::cout << "main exit()" << std::endl;

    return 0;
}