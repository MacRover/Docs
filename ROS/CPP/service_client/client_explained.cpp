# Client Explained

We first need to create the `add_two_ints`. The `ros::ServiceClient` object is used to call the service later.

```c++
ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints")
```

We create the srv object which contains a `request` and `response`.

```c++
beginner_tutorials::AddTwoInts srv
```

We then need to define the request parameters. We take in two strings from the command line and covert them to long long integers.

```c++
srv.request.a = atoll(argv[1]);
srv.request.b = atoll(argv[2]);
```

Now we need to make sure that we actually send the call. We do this through `call(<service>)`. Since the calls are blocking meaning we wait the `client.call(<service>)` to return true meaning the call has been returned. If the call did not succed, it would return false and we value in `srv.response` would be invalid.
