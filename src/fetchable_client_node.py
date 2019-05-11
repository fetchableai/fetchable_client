#!/usr/bin/env python

from fetchable_client.srv import Fetch, FetchResponse
import rospy
from fetchable import FetchableClient
import json

auth_file = rospy.get_param('fetchable_auth_path')

if(auth_file):
    client = FetchableClient(user_agent="fetchable-ros-client/0.1.0", auth_file=auth_file)
else:
    client = FetchableClient(user_agent="fetchable-ros-client/0.1.0")


def fetch_request(req):
    if(not req.endpoint):
        response_string = json.dumps({ "status_code": 2001, "reason": "Empty endpoint"})
        return FetchResponse(response=response_string)

    response = client.fetchEndpoint(req.endpoint)
    return FetchResponse(response=json.dumps(response))



def fetchable_client():
    rospy.init_node('fetchable_client')
    service = rospy.Service('fetch', Fetch, fetch_request)
    rospy.spin()

if __name__ == "__main__":
    fetchable_client()
