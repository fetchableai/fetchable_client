#include <string>
#include <iostream>

#include "ros/ros.h"
#include "fetchable_client/Fetch.h"


int main(int argc, char **argv) {

  ros::init(argc, argv, "example_caller_cpp");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<fetchable_client::Fetch>("fetch");

  std::string endpoint;
  std::cout << "enter endpoint (or quit): " << std::endl;
  std::cin >> endpoint;

  while(endpoint.compare("quit")) {

    if(client.exists()) {

      fetchable_client::Fetch srv;
      srv.request.endpoint = endpoint;

      if(client.call(srv)) {
        std::cout << srv.response.response << std::endl;

      } else {
        std::cout << "service call failed" << std::endl;
      }

    } else {
      std::cout << "service not available" << std::endl;
    }


    std::cout << std::endl << "enter endpoint (or quit): " << std::endl;
    std::cin >> endpoint;

  }


  return 0;

}
