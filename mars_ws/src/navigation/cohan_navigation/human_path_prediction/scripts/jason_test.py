import json
import math

class RoomReader:
    def __init__(self, json_file):
        self.goals_x = []
        self.goals_y = []
        self.resolution = 0.5  
        self.read_json(json_file, self.resolution)
        # Print the extracted x and y values
        print("Goals X:", self.goals_x, len(self.goals_x))
        print("Goals Y:", self.goals_y, len(self.goals_y))

    def read_json(self, json_file, resolution):
        with open(json_file, 'r') as file:
            data = json.load(file)
            if "rooms" in data:
                rooms = data["rooms"]
                for room_key, room_data in rooms.items():
                    if "outline" in room_data:
                        outline = room_data["outline"]
                        if len(outline[0]) > 0:
                            x, y, _ = outline[0][0]
                            self.goals_x.append(x)
                            self.goals_y.append(y)

                            for point in outline[0][1:]:
                                x, y, _ = point
                                prev_x, prev_y = self.goals_x[-1], self.goals_y[-1]

                                # Check the distance to the previous point
                                distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)

                                # Add points along the outline if distance exceeds resolution
                                while distance > resolution:
                                    angle = math.atan2(y - prev_y, x - prev_x)
                                    new_x = prev_x + resolution * math.cos(angle)
                                    new_y = prev_y + resolution * math.sin(angle)
                                    self.goals_x.append(new_x)
                                    self.goals_y.append(new_y)
                                    prev_x, prev_y = new_x, new_y
                                    distance = math.sqrt((x - prev_x)**2 + (y - prev_y)**2)

                                self.goals_x.append(x)
                                self.goals_y.append(y)

if __name__ == "__main__":
    json_file_path = "crossing_corrider_outlines.json"  # Replace with the actual path to your JSON file
    resolution_value = 0.5  # Replace with your desired resolution
    room_reader = RoomReader(json_file_path)

    
