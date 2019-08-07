#include "camera_manager.hpp"


int main(int argc, char * argv[])
{

    ros::init(argc, argv, "camera_manager");

    auto camera_ns_list = CameraParam::loadCameras();

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
            std::cerr << "Initialise SN: " << camera_ns << " Failed" << std::endl;
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
    ros::spin();

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