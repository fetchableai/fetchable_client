=================================
Fetchable: ROS Package
=================================

This is a pre-built Fetchable package for ROS designed to make is super easy for robots to search the world's information. It is written in Python and contains a single node, this node serves a single service whereby callers can specify the endpoint to make a request to. The response is then returned to the caller. Developers may also integrate Fetchable into their codebase by using the vanilla Python client-side SDK.


Table of Contents
=================

-  `Description <#description>`__
-  `Installation <#installation>`__
-  `Usage <#usage>`__
-  `Contributing <#contributing>`__
-  `License <#license>`__

Description
============

The package contains a single node 'fetchable_client' and expose a single service :code:`/fetch`. The service message type is called Fetch and contains a single request and response field.

.. code-block::
  string endpoint
  ---
  string response

The endpoint specifies the exact endpoint you wish to access, e.g. :code:`/status`, :code:`/random/joke`, :code:`/v0.1/amazon_river/length`. The response contains the JSON response from the server (or error message from the Fetchable Python client-side library if it can't access the server) encoded in a string format. It is up to the caller of the service to decode it into a JSON object.

The package uses the Fetchable Python client-side SDK (found `here
<https://github.com/fetchableai/fetchable-python>`_) to make the calls under the hood and the reader should familiarise themselves with how this works.


Installation
============

There are two methods for installing this package: (1) building from source, or (2) installing binary release. Building from source allows you to make modifications to the code yourself and extend it. Installing the binary package doesn't offer you that but lets you start fetching data quicker.

1. Common Step (Pre-requisites)
-------------------------------

* Python 2.7 or 3.4
* An active Fetchable account.


2. Building from source
-----------------------

a. Install the Fetchable Python client-side SDK.

.. code-block:: sh

  $ pip install fetchable-client

b. Get the source code.

.. code-block:: sh

  $ cd <ros-workspace>/src
  $ git clone https://github.com/fetchableai/fetchable_client.git
  $ cd ../
  $ catkin_make


2. Installing binary release
----------------------------

.. code-block:: sh

  $ sudo apt-get install ros-<distro>-fetchable-client


3. Common Step (env. variables)
----------------------------

You will need to provide the path to the file containing your authentication keys. There are two ways to specify the path to your account's authentication file. Firstly, you may set it as an environment variable and the package will automatically read it.

.. code-block:: sh

  $ export FETCHABLE_AUTH_FILE=/path/to/file.json

The second way is to specify it through the launch file when launching like so:

.. code-block:: xml

  $ roslaunch fetchable_client client.launch fetchable_auth:=/path/to/file.json



Usage
=====

Running the Node
----------------




Calling the service (Command Line)
----------------------------------

The service can be called with the following command:


.. code-block:: sh

  $ rosservice call /fetch "endpoint: '/random/joke'"



Calling the service (Through code)
----------------------------------

The service can be called through code in other nodes. There are plenty of examples of how to do this online but to make things easier we have included an example in python and c++ in the :code:`/tests` folder. They can be run with :code:`rosrun fetchable_client example_caller_py` and :code:`rosrun fetchable_client example_caller_cpp` respectively. Each code will repeatedly prompt the user to enter an endpoint through the terminal until 'quit' is entered.

The most basic version in Python looks like this.

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
