# YASKAWA FS100 High Speed Ethernet Server Functions
  This class implements most of YASKAWA FS100 High Speed Ethernet Server Functions.  
  https://www.motoman.com/getmedia/16B5CD92-BD0B-4DE0-9DC9-B71D0B6FE264/160766-1CD.pdf.aspx?ext=.pdf  
  It also works with the YRC1000; command layouts are checked against the YRC1000 Ethernet
  function manual HW1483358 rev 11, chapter 3.

  Attributes:  
      ip (str): IP address of the controller  
      timeout (int, optional): Communication timeout value in second between PC and Controller. Defaults to 0.8.  
      retries (int, optional): Resends of a robot control request after a timeout. Defaults to 0.  
      port_robot_control, port_file_control (int, optional): UDP ports, default 10040 / 10041.  
      errno (int): Number of last error  

  Methods:  
      switch_power(): Turn on/off the power supply  
      mov(): Make robot move to a specified position  
      pmov(): Make robot move to a specified pulse position  
      select_cycle(): Select the way a job in pendant plays  
      select_job(): Select a job in pendant for later playing  
      play_job(): Start playing a job in pendant  
      read_executing_job_info(): Read the info of executing job (master or sub task)  
      read_axis_name(): Read the name of each axis  
      read_position(): Read the robot position  
      read_position_error(): Read the robot position error data  
      read_torque(): Read the robot torque data of each axis  
      read_encoder_temperature(): Read the encoder temperature of each axis (YAS4.81+)  
      read_regenerative_power(): Read the amount of regenerative power (YAS4.90+)  
      read_converter_temperature(): Read the converter temperature (YAS5.20+)  
      read_variable(): Read a robot variable  
      read_variables(): Read multiple robot variables with plural commands  
      write_variable(): Write a robot variable  
      write_variables(): Write multiple consecutive robot variables with plural commands  
      get_status(): Retrieve various status of the robot  
      get_last_alarm(): Retrieve info of the latest alarm  
      read_alarm() / read_alarm_ex(): Retrieve one of the four current alarms (ex: with sub code strings)  
      read_alarms(): Retrieve all current alarms  
      read_alarm_info() / read_alarm_info_ex(): Retrieve one alarm history entry  
      read_alarm_history(): Retrieve one alarm history category  
      reset_alarm(): To reset alarms or cancel errors  
      acquire_system_info(): Acquire system information  
      acquire_management_time(): Acquire usage time of an action  
      show_text_on_pendant(): Show text on pendant  
      get_file_list(): Retrieve list of files ended with extension in pendant  
      send_file(): Send a local file to pendant  
      recv_file(): Receive a file from pendant  
      read_file(): Receive a file from pendant into memory  
      delete_file(): Delete a file in pendant  
      request(): Send any robot control command, get the raw answer  
      describe_error(): Text for a status / added status pair  

  Diagnostics go to the `fs100` logger (`logging`), not to stdout.  
  Set `FS100.ENCODING` to the pendant language encoding for non-ASCII strings.

# Simulator
  `fs100_sim.py` answers the commands above with plausible data on UDP 10040/10041:  
  $ ./fs100_sim.py --host 127.0.0.2

# Test
  Offline, against the simulator:  
  $ ./tests/test_fs100_offline.py  
  Against a real controller (writes variables, moves the robot!):  
  $ ./tests/test_fs100.py  
