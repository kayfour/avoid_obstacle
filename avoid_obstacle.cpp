#include <iostream>
#include <chrono>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>
#include <future>
#include <math.h>
using namespace mavsdk;
#define PI 3.14159265358979323846

// 포인트를 입력하면 미션을 만들어 push back
void make_mission_point(std::vector<Mission::MissionItem> &mission_items, double x, double y){
    Mission::MissionItem mission_item;
    mission_item.latitude_deg = x;    // 범위: -90 to +90
    mission_item.longitude_deg = y;    // 범위: -180 to +180
    mission_item.relative_altitude_m = 10.0f;    // takeoff altitude
    mission_item.speed_m_s = 27.77777777777777777777777777778f; //단위 m/s, 시속 100km/s
    mission_item.is_fly_through = true;   
    mission_items.push_back(mission_item);
}
// 장애물 지점의 좌표를 입력하고 진입 각도(seta, radian)를 입력하면 
// 장애물을 중심으로 일정 거리를 두고 반원으로 비행하며 회피
void collision_avoidance_point(std::vector<Mission::MissionItem> &mission_items, 
                        double point_x_obstacle,            // 장애물의 x 좌표
                        double point_y_obstacle,            // 장애물의 y 좌표
                        double seta,                        // 진입 각도, radian
                        int mision_num = 15,                // 미션의 수
                        double radius_avoidance = 0.00015)   // 장애물 중심부로부터 거리
                        {  
    int max = mision_num;
    double tmp_x, tmp_y, r=radius_avoidance;  // 원의 임시 포인트와 반지름(물체 중심점과 유지하는 거리)

    for(int i=0; i<max+1; i++){
        tmp_x = point_x_obstacle - r*sin(seta + i*PI/max ); // 한바퀴면 (360*i/max) * PI / 180
        tmp_y = point_y_obstacle - 1.5*r*cos(seta + i*PI/max ); // 하지만 반바퀴이므로 (180*i/max) * PI / 180
        make_mission_point(mission_items, tmp_x, tmp_y);
    }
}

int main(int argc, char** argv)
{   //==============================================================================================
    // connect_result
    //==============================================================================================
    Mavsdk mavsdk;
    ConnectionResult connection_result;
    bool discovered_system = false;

    connection_result = mavsdk.add_any_connection(argv[1]);

    if (connection_result != ConnectionResult::Success) {return 1;}

    mavsdk.subscribe_on_new_system([&mavsdk, &discovered_system]() {    // 콜백함수 with 익명함수
        const auto system = mavsdk.systems().at(0);
        if (system->is_connected()) {discovered_system = true;} // 시스템 연결 확인
    });

    std::this_thread::sleep_for(std::chrono::seconds(2));   // 콜백함수에 의한 시스템 연결 대기

    if (!discovered_system) {return 1;} // 시스템 디텍트 실패시 리턴 1
    const auto system = mavsdk.systems().at(0);

    system->register_component_discovered_callback(
        [](ComponentType component_type) -> void {std::cout << unsigned(component_type);}
    );
    //==============================================================================================
    // regist_telemetry
    //==============================================================================================
    auto telemetry = std::make_shared<Telemetry>(system);   // 원격 측정

    const Telemetry::Result set_rate_result = telemetry -> set_rate_position(1.0);  // 1초마다 수신
    if(set_rate_result != Telemetry::Result::Success){return 1;}

    double lat, lon;
    telemetry -> subscribe_position([&lat, &lon](Telemetry::Position position){   // 고도를 모니터링하기 위한 콜백함수, 익명함수
            lat = position.latitude_deg;    // 현재 좌표 
            lon = position.longitude_deg;
            
            std::cout << "Altitude: " << position.relative_altitude_m << "m "; 
            std::cout << "lat: " << lat << " deg ";
            std::cout << " lon: " << lon << " deg " << std::endl; 
    });
    while (telemetry -> health_all_ok() != true){
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    //==============================================================================================
    // Final Report
    //==============================================================================================
    std::vector<Mission::MissionItem> mission_items;
    Mission::MissionItem mission_item;
    const double center_lat=lat, center_lon=lon; // 기준점, 현재 위치의 좌표를 받아 기준점을 만든다.
      
    double point_x1 = center_lat         , point_y1 = center_lon + 0.001;
    double point_x2 = center_lat         , point_y2 = center_lon + 0.002;    
    
    // 장애물 포인트, 임시로 포인트1과 포인트2 사이에 지정
    double point_x_obstacle = point_x1 + 0.5*( point_x1 - point_x2), point_y_obstacle = point_y1 + 0.5*(point_y2 - point_y1);  
    
    make_mission_point(mission_items, point_x1, point_y1);  
    double seta = 0.0001;  // 진입각도,  radian
    collision_avoidance_point(mission_items, point_x_obstacle, point_y_obstacle, seta); // 장애물 회피
    make_mission_point(mission_items, point_x2, point_y2);             

    //==============================================================================================
    // upload mission
    //==============================================================================================
    auto mission = std::make_shared<Mission>(system);   
    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom -> get_future();  // 결과를 가져온다
        Mission::MissionPlan mission_plan;
        mission_plan.mission_items = mission_items;
        mission->upload_mission_async(mission_plan,
                                    [prom](Mission::Result result){
                                        prom -> set_value(result);
                                    });
        const Mission::Result result = future_result.get();
        if(result != Mission::Result::Success){return 1;}
    }
    //==============================================================================================
    // arm
    //==============================================================================================
    auto action = std::make_shared<Action>(system);
    std:: cout << "Arming..." << std::endl;
    const Action::Result arm_result = action -> arm();  // 암

    if(arm_result != Action::Result::Success){
        std::cout << "Arming failed: " << arm_result <<std::endl;
        return 1;
    }
    //==============================================================================================
    // mission progress
    //==============================================================================================
    {
        std:: cout << "Starting mission." << std::endl;
        auto start_prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = start_prom -> get_future();
        mission->start_mission_async([start_prom](Mission::Result result){
            start_prom -> set_value(result);
            std::cout << "Started mission." << std::endl;
        });
        const Mission::Result result = future_result.get();
        if(result != Mission::Result::Success){return 1;}
        while (!mission -> is_mission_finished().second){
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
     
    std::this_thread::sleep_for(std::chrono::seconds(10)); //stuck air
    std::cout << "Landing..." << std::endl;
    const Action::Result land_result = action -> land();
    if (land_result != Action::Result::Success){
        return 1;
    } // 착륙 실패           
    
    while (telemetry->armed()) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Disarmed..." << std::endl;
    std::cout << "Finishied" << std::endl;

    return 0;
}