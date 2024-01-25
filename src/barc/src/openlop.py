import csv, rospy
from std_msgs.msg import Float64


if __name__ == '__main__':
    rospy.init_node('FAKE_SEND', anonymous=True)

    csv_file_path = 'inputs_offset.csv'

    # Create an empty list to store the contents
    csv_data = []

    # Open the CSV file and read its contents
    with open(csv_file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        
        # Iterate over each row in the CSV file
        for row in reader:
            # Append the row to the list
            row[0] = float(row[0])
            row[1] = float(row[1])
            
            csv_data.append(row)
    
    steer_pub = rospy.Publisher('/steering', Float64, queue_size=10)
    throttle_pub = rospy.Publisher('/throttle', Float64, queue_size=10)

    for i in csv_data:
        steer_pub.publish(i[1])
        throttle_pub.publish(i[0])
        rospy.Rate(10).sleep()

    # rospy.spin()
