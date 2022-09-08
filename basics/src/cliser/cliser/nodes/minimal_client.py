#!/usr/bin/python3
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, "/mae_tutorials/add_two_ints")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self):

        numbers=[]

        while len(numbers) < 2:
            try:
                data = int(input("Enter a Integer: "))
                print("You entered: ", data)
                numbers.append(data)
            except ValueError:
                print("Invalid input - must be an integer!")

        self.req.a = numbers[0]
        self.req.b = numbers[1]
        self.future = self.cli.call_async(self.req)
        
        # here, we are asking ourselves to wait until the call is complete!
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClient()
    response = minimal_client.send_request()
    minimal_client.get_logger().info('Result of add_two_ints: ' + str(response.sum))

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()