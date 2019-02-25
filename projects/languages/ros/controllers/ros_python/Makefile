ROS_PYTHON_ZIP = ros_python.zip
ROS_PYTHON_DEPENDENCY_URL = https://cyberbotics.com/files/repository/dependencies/common/$(ROS_PYTHON_ZIP)

release debug profile: kinetic python

kinetic python:
	@echo "# downloading and installing rospy dependencies"
	@wget -q $(ROS_PYTHON_DEPENDENCY_URL)
	@unzip -q -o $(ROS_PYTHON_ZIP)
	@rm $(ROS_PYTHON_ZIP)

clean:
	@rm -fr kinetic python
