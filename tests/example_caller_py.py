#!/usr/bin/env python

from fetchable_client.srv import Fetch
import rospy



def main():
    rospy.init_node('example_caller_py')

    rospy.wait_for_service('fetch')
    fetch_service = rospy.ServiceProxy('fetch', Fetch)

    endpoint = raw_input("enter endpoint (or quit): ")
    while(endpoint != "quit"):
        print(endpoint)
        try:
          response = fetch_service(endpoint)
          print(response)

        except rospy.ServiceException as exc:
          print("Service did not process request: " + str(exc))

        endpoint = raw_input("enter endpoint (or quit): ")










if __name__ == "__main__":
    main()
