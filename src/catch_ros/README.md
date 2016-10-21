
catch_ros
=========

catch_ros is a small ROS catkin wrapper around the very nice [Catch][1] unit
testing framework by Phil Nash.

[1]: https://github.com/philsquared/Catch

Usage
-----

Usage in `CMakeLists.txt`:

```cmake
find_package(catkin REQUIRED COMPONENTS
	...
	catch_ros
	...
)

# Variant 1: standalone test
catch_add_test(my_standalone_test
	test/my_test.cpp
)
target_link_libraries(my_standalone_test
	${catkin_LIBRARIES}
)

# Variant 2: test node used in a rostest file
catch_add_rostest_node(my_rostest_test
	test/my_test.cpp
)
target_link_libraries(my_rostest_test
	${catkin_LIBRARIES}
)
```

The my_standalone_test is added to the `run_tests` target, so it is run by
the catkin test infrastructure. my_rostest_test is not added to `run_tests`,
instead you can use it in a rostest file.

`test/my_test.cpp`:

```C++
#include <catch_ros/catch.hpp>

TEST_CASE("test_case", "[some tag]")
{
	REQUIRE( (1 + 1) == 2 );
}
```

License
-------

The catch_ros wrapper is released under BSD-3. Catch is licensed under
the Boost license (see the Catch repository for details). We provide the
single header version for convenience in this repository at
`include/catch_ros/catch.hpp`.

Contact
-------

If you have any questions, mail Max Schwarz (max.schwarz@uni-bonn.de).
