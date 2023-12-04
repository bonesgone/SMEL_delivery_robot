#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
import requests  # Import the requests library
from math import radians, sin, cos, sqrt, atan2

class OrderHandler(Node):
    def __init__(self):
        super().__init__('order_handler')
        self.declare_parameter('api_url', 'https://jsonplaceholder.typicode.com/posts/1')
        self.declare_parameter('api_token', '939fde5844135fa7046d430c44f5050bfc4b129d')

        self.api_url = self.get_parameter('api_url').value
        self.api_token = self.get_parameter('api_token').value
        response = self.get_base_token(self.api_token)
        self.declare_parameter('base_token', response['base_token'])
        self.declare_parameter('base_uuid', response['base_uuid'])
        self.base_token = self.get_parameter('base_token').value
        self.dtable_uuid = self.get_parameter('base_uuid').value
        self.robot_location = '42.845303745425674, 74.60259336530775'
        # self.timer = self.create_timer(1.0, self.timer_callback)
        # self.timer = self.create_timer(1.0, self.get_base_token)
        self.orders = self.get_orders(self.dtable_uuid, self.base_token, 'Orders')
        best_order = self.find_min_distance_order()
        if best_order is not None:
            self.get_logger().info(f'Best Order: {best_order}')
        else:
            self.get_logger().info('No orders available')


    def get_base_token(self, api_token):
        response = requests.get(
            url='https://cloud.seatable.io/api/v2.1/dtable/app-access-token/',
            headers={'Authorization': f'Bearer {api_token}',
                 "accept": "application/json"}
        )
        base_uuid = response.json()['dtable_uuid']
        base_token = response.json()['access_token']
        self.get_logger().info(f'Base UUID: {base_uuid}')
        self.get_logger().info(f'Base token: {base_token}')
        return {'base_uuid': base_uuid, 'base_token': base_token}


    def get_orders(self, base_uuid, base_token, table_name):
        url = f"https://cloud.seatable.io/dtable-server/api/v1/dtables/{base_uuid}/rows/?table_name={table_name}"
        self.get_logger().info(f'Sending HTTP Request to: {url}')
        headers = {
            "accept": "application/json",
            "authorization": f"Bearer {base_token}"
        }

        response = requests.get(url, headers=headers)
        self.get_logger().info(f'JSON: {response.json()}')
        orders = response.json()['rows']
        return orders


    def parse_location_string(self, location_str):
        # Parse location string into latitude and longitude
        lat_str, lon_str = location_str.split(',')
        return float(lat_str), float(lon_str)


    def calculate_distance(self, point1, point2):
        # Haversine formula for distance calculation
        R = 6371.0  # Radius of the Earth in kilometers
        lat1, lon1 = point1
        lat2, lon2 = point2
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        dlon = lon2 - lon1
        dlat = lat2 - lat1

        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))

        distance = R * c
        return distance

    def find_min_distance_order(self):
        min_distance = float('inf')
        best_order = None
        robot_coords = self.parse_location_string(self.robot_location)

        for order in self.orders:
            client_coords = self.parse_location_string(order['client_location'])
            order_distance = self.calculate_distance(robot_coords, client_coords)

            if order_distance < min_distance:
                min_distance = order_distance
                best_order = order['_id']

        return best_order


def main(args=None):
    rclpy.init(args=args)
    node = OrderHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
