=================================
Fetchable: ROS Package
=================================

This is the Fetchable package for ROS. It allows developers to make retrieve data from the Fetchable API and make it available to the rest of a ROS system. It is written in Python and contains a single node. This node serves a single service whereby callers can specify the endpoint to make a request to. The response is then returned to the caller.


Table of Contents
=================

-  `Description <#description>`__
-  `Usage <#usage>`__
-  `Contributing <#contributing>`__
-  `License <#license>`__

Description
============

The package contains a single node 'fetchable_client' and expose a single service `/fetch`. The service message type is called Fetch and contains a single request and response field.

```
string endpoint
---
string response

```

The endpoint specifies the exact endpoint you wish to access, e.g. `/status`, `/random/joke`, `/v0.1/amazon_river/length`. The response contains the JSON response from the server (or error message from the Fetchable Python client-side library if it can't access the server) encoded in a string format. It is up to the caller of the service to decode it into a JSON object.

The package uses the Fetchable Python client-side SDK (found `here
<https://github.com/fetchableai/fetchable-python>`_) to make the calls under the hood and the reader should familiarise themselves with how this works.


Usage
=====

Running the Node
----------------

There are two ways to specify the path to your account's authentication file. Firstly, you may set it as an environment variable and the package will automatically read it.

.. code-block:: sh

  $ export FETCHABLE_AUTH_FILE=/path/to/file.json

The second way is to specify it through the launch file when launching like so:

.. code-block:: xml

  $ roslaunch fetchable_client client.launch fetchable_auth:=/path/to/file.json




Calling the service
-------------------

There are two ways to call the service.

DESCRIBE THE SERVICE

1. Command line
~~~~~~~~~~~~~~~

.. code-block:: sh

  $ rosservice call /fetch "endpoint: '/random/joke'"


1. Through code
~~~~~~~~~~~~~~~

The service can be called through code in other nodes. There are plenty of examples of how to do this online but to make things easier we have included an example in python and c++ in the `/tests` folder. They can be run with `rosrun fetchable_client example_caller_py` and `rosrun fetchable_client example_caller_cpp` respectively. Each code will repeatedly prompt the user to enter an endpoint through the terminal until 'quit' is entered.

The most basic version looks like this.

.. code-block:: python

  from fetchable_client.srv import Fetch

  #...

  rospy.init_node('example_caller_py')

  rospy.wait_for_service('fetch')
  fetch_service = rospy.ServiceProxy('fetch', Fetch)

  response = fetch_service('/random/joke')

  #...

And in c++, looks like this

.. code-block:: cpp

  #include "fetchable_client/Fetch.h"

  //...

  ros::init(argc, argv, "example_caller_cpp");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<fetchable_client::Fetch>("fetch");

  //...

  fetchable_client::Fetch srv;
  srv.request.endpoint = endpoint;

  client.call(srv);

  std::cout << srv.response.response << std::endl;

  //...




Contributing
============

Contributions are welcome and encouraged! See the `Contributing Guide <CONTRIBUTING.rst>`_ for information on how to contribute.


License
=======
Licensed under Apache Version 2.0.

See the `LICENSE <LICENSE>`_ file for more information.
