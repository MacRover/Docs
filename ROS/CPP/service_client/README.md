# Service and Client


Services are meant for non-critical application of one-to-one communication between two nodes. We can see this is different from topics where we have many-to-many communcation.

Services are developed on a request and response whereas topics are seen as a black box. It is useful for services for data processing that multiple nodes of a package requires rather than rewriting the code in each node.

The setup for the package.xml and CMakeLists.txt is the same as topics/subscribers.
