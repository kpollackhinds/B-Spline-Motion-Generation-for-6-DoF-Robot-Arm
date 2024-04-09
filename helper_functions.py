
def parse_pose(file_name = None):
    if not file_name:
        return 
    poses = []
    with open(file_name, 'r') as file:
        for line in file:
            int_pose = [int(val) for val in line.split(',')]
            poses.append(int_pose)
            print(poses)
            # continue
    return poses

def run_motion():
    parse_pose()

if __name__ == "__main__":
    parse_pose("test.txt")