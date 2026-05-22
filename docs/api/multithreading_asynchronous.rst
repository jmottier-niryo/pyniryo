=====================================================
PyNiryo Multithreading & Asynchronous API Documentation
=====================================================

Overview & Use Case
-------------------
This functionality within PyNiryo is designed for users who need to control their robot in asynchronous mode or utilize multiple threads. It is particularly useful for parallelizing tasks, such as reading an input/output (IO) sensor in one thread while simultaneously moving the robot in another. 

This update bridges the gap between ``pyniryo`` and the deprecated ``pyniryo2``, providing a single, centralized library that satisfies complex, concurrent use cases.

.. warning::
   **Prerequisite:** To use this functionality, your Ned stack must be on version **6.2.0 or newer**.

How to Activate
---------------
By default, multithreading and asynchronous modes are disabled. When initializing the ``NiryoRobot`` object, you can enable them by setting the respective parameters to ``True``:

.. code-block:: python

    from pyniryo import NiryoRobot

    # Activating both features during initialization
    robot = NiryoRobot(ip_address='169.254.200.200', multithreading=True, asynchronous=True)

For reference, here are the default values when initializing a Niryo robot without explicit parameters:

.. code-block:: python

    NiryoRobot(ip_address=None, verbose=True, logger=None, multithreading=False, asynchronous=False)

Deep Dive: Core Concepts
------------------------
To understand the changes in PyNiryo, let's explore Multithreading and Asynchronous modes separately. (Note: These modes are closely related and can be used together).

1. Multithreading
^^^^^^^^^^^^^^^^^
Once the ``multithreading`` variable is set to ``True``, you can create and manage as many threads as your application requires. There is no special syntax or new function names to learn; you write your code as you normally would, but you can now execute commands concurrently across multiple threads.

**Example: Real-time Position Monitoring**
In this example, one thread commands the robot to move, while a second thread continuously reads and prints the joint and pose positions in real-time.

.. code-block:: python

    from pyniryo import *
    import threading

    robot = NiryoRobot(ip_address='169.254.200.200', multithreading=True)
    robot.calibrate_auto()

    def move_it():
        robot.clear_collision_detected()
        i = 1
        while True:
            robot.move(JointsPosition(0, 0, i, 1.09, 0, 0))
            i *= -1

    def print_it():
        while True:
            print(robot.get_joints())
            print(robot.get_pose())

    # Initialize daemon threads
    thread1 = threading.Thread(target=print_it, daemon=True)
    thread2 = threading.Thread(target=move_it, daemon=True)

    # Start threads
    thread1.start()
    thread2.start()

    # Keep main thread alive
    thread1.join()
    thread2.join()

    robot.quit()

2. Asynchronous Mode
^^^^^^^^^^^^^^^^^^^^
By setting ``asynchronous=True``, functions that take time to execute will no longer block your main program loop. Instead of returning the final result immediately, these functions return a **Future** object.

What you can do with the Future object:

* ``.value(timeout)``: Retrieves the result of the command. The ``timeout`` parameter (in seconds) is optional. If the result is not yet ready, calling this method will block execution until the command finishes.
* ``.is_done()``: Returns a boolean. ``True`` means the command has finished executing, while ``False`` means it is still in progress.

*Note: Asynchronous mode applies only to time-consuming functions. Instantaneous functions remain synchronous.*

**Functions that return a Future object:**

* ``move_to_home_pose()``
* ``move_to_object()``
* ``vision_pick()``
* ``move()``
* ``joints()``
* ``pose()``

**Example: Basic Asynchronous Call**

.. code-block:: python

    from pyniryo import *

    # Initialize with asynchronous mode enabled
    robot = NiryoRobot(ip_address='169.254.200.200', asynchronous=True)

    # The move command returns a Future object immediately
    r_future = robot.move(JointsPosition(0, 0, 0, 0, 0, 0))
    
    # We can check positions while the robot is moving
    print(robot.get_joints())
    
    # Block and wait for the final result
    print(r_future.value())

*Note on compatibility: When you activate asynchronous mode, you can also utilize standard Python multithreading without any additional configuration.*

Timeout Control
---------------
A new feature introduced alongside multithreading and asynchronous mode is the ability to apply a ``timeout`` to the functions you use in multithreading mode. This parameter dictates the maximum time (in seconds) your program will wait for the robot to respond before raising a ``TimeoutError``.

**Usage:**

Simply call the ``set_multithreading_timeout`` function:

.. code-block:: python

    robot.set_multithreading_timeout(timeout=4.0)

Comprehensive Application Example
---------------------------------
To illustrate how these features come together in a real-world scenario, the following example demonstrates a robotic arm managing traffic between two conveyor belts equipped with IR sensors. 

It utilizes multithreading to monitor both sensors simultaneously while managing a centralized job queue for the robot arm.

.. code-block:: python

    from pyniryo import *
    import threading
    import queue
    import time

    hostname = "192.168.1.223"
    
    # Initialize the robot with multithreading enabled directly in the constructor
    n = NiryoRobot(hostname, multithreading=True)

    job_queue = queue.Queue()

    # Pre-defined positions for Conveyor 1
    obs_pos_1 = [0.1, 0.0, 0.2, 0.0, 0.0, 0.0] 
    bin_pos_1 = [0.0, 0.2, 0.1, 0.0, 0.0, 0.0]

    # Pre-defined positions for Conveyor 2
    obs_pos_2 = [0.1, -0.1, 0.2, 0.0, 0.0, 0.0]
    bin_pos_2 = [0.0, -0.2, 0.1, 0.0, 0.0, 0.0]

    # Fetch hardware IDs
    conveyor_1, conveyor_2 = n.get_connected_conveyors_id()
    workspace_1, workspace_2 = n.get_workspace_list()

    def check_IR1_data():
        n.run_conveyor(conveyor_1)
        resume_conveyor_event = threading.Event() 
        
        while True:
            value = n.digital_read("DI5")
            if value == 1:
                n.stop_conveyor(conveyor_1)
                
                job = {
                    "obs_pos": obs_pos_1,
                    "bin_pos": bin_pos_1,
                    "workspace": workspace_1,
                    "resume_event": resume_conveyor_event
                }
                job_queue.put(job)
                
                # Wait for the robot to finish handling this conveyor's item
                resume_conveyor_event.wait()
                resume_conveyor_event.clear()
                
                n.run_conveyor(conveyor_1)
                
            time.sleep(0.1)

    def check_IR2_data():
        n.run_conveyor(conveyor_2) 
        resume_conveyor_event = threading.Event()
        
        while True:
            value = n.digital_read("DI1")
            if value == 1:
                n.stop_conveyor(conveyor_2)
                
                job = {
                    "obs_pos": obs_pos_2,
                    "bin_pos": bin_pos_2,
                    "workspace": workspace_2,
                    "resume_event": resume_conveyor_event
                }
                job_queue.put(job)
                
                # Wait for the robot to finish handling this conveyor's item
                resume_conveyor_event.wait()
                resume_conveyor_event.clear()
                
                n.run_conveyor(conveyor_2)
                
            time.sleep(0.1)

    def perform_pick_place():
        while True:
            # Block until a job is available in the queue
            current_job = job_queue.get()
            
            n.move(current_job["obs_pos"]) 
            n.vision_pick(current_job["workspace"])
            n.move(current_job["bin_pos"])
            n.release_with_tool()
            
            # Signal the waiting conveyor thread to resume
            current_job["resume_event"].set()
            job_queue.task_done()

    def main():
        # Instantiate and start threads
        thread_conv1 = threading.Thread(target=check_IR1_data, daemon=True)
        thread_conv2 = threading.Thread(target=check_IR2_data, daemon=True)
        thread_robot = threading.Thread(target=perform_pick_place, daemon=True)

        thread_conv1.start()
        thread_conv2.start()
        thread_robot.start()

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("Shutting down...")
            n.close_connection()

    if __name__ == "__main__":
        main()