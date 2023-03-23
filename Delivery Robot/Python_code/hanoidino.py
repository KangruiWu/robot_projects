import cv2 as cv
import numpy as np
from urllib.request import urlopen
import face_recognition
from green_cube import GreenCube
from red_cube import RedCube
from orange_cube import orangeCube
from blue_cube import BlueCube
from statistics import mean
from people import People
import subprocess
import serial
import time


MAX_BUFF_LEN = 255
SETUP = False 

#connecting to serial port
prev = time.time()
while(not SETUP):
	try:
		port = serial.Serial("/dev/ttyUSB0", 115200, timeout=None)

	except serial.serialutil.SerialException:
		#subprocess.run("fuser /dev/ttyUSB1")
		port = None
		if(time.time() - prev > 2): # Don't spam with msg
			print("No serial detected, please plug your uController")
			prev = time.time()
	except:
		port = None
		if(time.time() - prev > 2): # Don't spam with msg
			print("No serial detected, please plug your uController")
			prev = time.time()

	if(port is not None): # We're connected
		SETUP = True


	pierrick_reco = face_recognition.load_image_file("people/pierrick.jpg")
	ray_reco = face_recognition.load_image_file("people/ray.jpg")
	matthew_reco = face_recognition.load_image_file("people/woodward.jpg")

	pierrick = People("pierrick")
	ray = People("ray")
	matthew = People("matthew")

	know_people = {"Pierrick": pierrick, "Ray": ray, "Matthew": matthew}
	pierrick_face = face_recognition.face_encodings(pierrick_reco)[0]
	ray_face = face_recognition.face_encodings(ray_reco)[0]
	matthew_face = face_recognition.face_encodings(matthew_reco)[0]

	known_face_encodings = [
		pierrick_face,
		ray_face,
		matthew_face,
	]
	known_face_names = [
		"Pierrick",
		"Ray",
		"Matthew",
	]

	roles = ["platform1","platform2","platform3"]
	sizes = {"small":"blue", "medium":"green", "large":"orange"}




def brain():
	# main cognitive function, get initial state (coöputer vision) builds the plan and uses to choose the next action to take, maps it to an ESP32 commands, 
	# send it and wait for return status
	# Keeps track of changes in the states environment (memory of who had which cube)
	print("\nInto brain\n")
	known_people = get_initialState()
	run_script = "./lilotane Hanoi/domain.hddl Hanoi/pfile01.hddl -v=0"#| cut -d' ' -f2- | sed 1,2d | head -n -2 > plan.txt
	output = subprocess.getoutput(run_script)
	plan = []
	for action in output.split("\n")[2:]:
		if ("root" not in action):
			digit = 0
			for i in action:
				if i.isdigit():
					digit += 1
				else:
					break
			action = action[digit:]
			plan.append(''.join([i for i in action])[1:]) # if not i.isdigit()
		else:
			break
	print("n------------------------------The plan is: ----------------------------\n")
	print(plan)

	angles_list = []

	while len(plan):    
		action = plan.pop(0).split(" ")
		print("\n\nNext action is: ", action)
		print("\n\nNext action is detect large")
		cmd = int(computerVision("large"))

		if action[0] == "load":
			print("\n\nNext action is detect: ", action[2])
			cmd = int(computerVision(action[2]))
			while type(cmd) == str:
				cmd = "t0040"
				print("Sending angle command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)
				cmd = computerVision()
			print(cmd)
			if cmd > 0:
				angle = "{0:0=3d}".format(cmd)
			else:
				angle = "{0:0=3d}".format(cmd)
			print(angle)
			cmd = "t" + angle
			print("Sending angle command to ESP: ", cmd)
			write_ser(cmd)
			string = read_ser(MAX_BUFF_LEN)
			if(len(string)):
				print(string)

			#time.sleep(1)
			print("\n\nNext action is GoTo: ", action[1])
			string = ""
			while not("done" in string):
				cmd = "g0160"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)

			#time.sleep(1)
			print("\n\nNext action is pick-up: ", action[1])
			string = ""
			while not("done" in string):
				cmd = "p0000"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)
			
			#time.sleep(1)
			print("\n\nNext action is back up")
			string = ""
			while not("done" in string):
				cmd = "b0160"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)

		elif action[0] == "unload":

			#time.sleep(1)
			print("\n\nNext action is detect: ", action[2])
			cmd = int(computerVision(action[2]))
			while type(cmd) == str:
				cmd = "t0040"
				print("Sending angle command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)
				cmd = computerVision()
			print(cmd)
			if cmd > 0:
				angle = "{0:0=3d}".format(cmd)
			else:
				angle = "{0:0=3d}".format(cmd)
			print(angle)
			cmd = "t" + angle
			print("Sending angle command to ESP: ", cmd)
			write_ser(cmd)
			string = read_ser(MAX_BUFF_LEN)
			if(len(string)):
				print(string)

			#time.sleep(1)
			print("\n\nNext action is GoTo: ", action[1])
			string = ""
			while not("done" in string):
				cmd = "g0160"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)
			
			#time.sleep(1)
			print("\n\nNext action is drop: ", action[1])
			string = ""
			while not("done" in string):
				cmd = "d0000"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)

			#time.sleep(1)
			print("\n\nNext action is back up")
			string = ""
			while not("done" in string):
				cmd = "b0160"
				print("Sending command to ESP: ", cmd)
				write_ser(cmd)
				string = read_ser(MAX_BUFF_LEN)
				if(len(string)):
					print(string)

			if action[2] in sizes:
				color2 = sizes[action[2]]
				color1 = sizes[action[1]]
				for person in know_people.keys():
					if color2 in know_people[person].has_cube.keys():
						person2 = person
					if color1 in know_people[person].has_cube.keys():
						person1 = person
			elif action[2] in roles:
				role2 = action[2]
				color1 = sizes[action[1]]
				for person in know_people.keys():
					if role2 in know_people[person].role:
						person2 = person
					if color1 in know_people[person].has_cube.keys():
						person1 = person

			print("before dictio cubes person1: ",know_people[person1].has_cube)
			print("before dictio cubes person2: ",know_people[person2].has_cube)
			know_people[person2].has_cube.update({color1: know_people[person1].has_cube[color1]})
			del know_people[person1].has_cube[color1]
			print("after dictio cubes person1: ",know_people[person1].has_cube)
			print("after dictio cubes person2: ",know_people[person2].has_cube)
			print()
	
		else:
			#time.sleep(1)
			cmd = "s0000"
			print("Sending command to ESP: ", cmd)
			write_ser(cmd)
			string = read_ser(MAX_BUFF_LEN)
			if(len(string)):
				print(string)
			

def get_initialState():
	# Starts computer vision to detect the inital state configuration (who has which cube at s0)
	# returns the state memory

	know_people = computerVision()
	while type(know_people) == str:
		cmd = "t0040"
		print("Sending angle command to ESP: ", cmd)
		write_ser(cmd)
		string = read_ser(MAX_BUFF_LEN)
		if(len(string)):
			print(string)
		know_people = computerVision()

	string = ""
	for people in know_people.keys():
		ordered_cubes = know_people[people].order_cubes()
		if len(ordered_cubes):
			for i in range(len(ordered_cubes)-1):
				string += "(on " + ordered_cubes[i+1].size + " " + ordered_cubes[i].size + ")"
			string += "(on " + ordered_cubes[0].size + " " + know_people[people].role + ")"
			string += "	(clear "+ordered_cubes[-1].size+")"
		else:
			string += "	(clear "+know_people[people].role+")"
	string += "\n"
	# with is like your try .. finally block in this case
	with open('Hanoi/pfile01.hddl', 'r') as file:
		# read a list of lines into data
		data = file.readlines()

	# now change the 2nd line, note that you have to add a newline
	data[10] = string

	# and write everything back
	with open('Hanoi/pfile01.hddl', 'w') as file:
		file.writelines( data )
	return know_people


def computerVision(aim=None):
	# starts reading the video stream from the ip, recognises people and cubes, returns the angle value based on the camera angle range if aim is not None

	#change ESP32-CAM ip

	"opening stream"
	url="http://10.245.156.72:81/stream"
	CAMERA_BUFFRER_SIZE=4096
	stream=urlopen(url)
	bts=b''
	i=0

	counter = 0
	counter_search = 0

	process_this_frame = True

	platform = 1

	x_span = 640#320
	y_span = 480#240
	
	finished = False
	while(not(finished)):
		#print("image")
		try:
			bts+=stream.read(CAMERA_BUFFRER_SIZE)
			jpghead=bts.find(b'\xff\xd8')
			jpgend=bts.find(b'\xff\xd9')
			if jpghead>-1 and jpgend>-1:
				#print("image")
				jpg=bts[jpghead:jpgend+2]
				bts=bts[jpgend+2:]
				img_big=cv.imdecode(np.frombuffer(jpg,dtype=np.uint8),cv.IMREAD_UNCHANGED)
				img=cv.resize(img_big,(x_span,y_span))

				#"""
				counter += 1
				if counter >= 14:
					counter = 0
				elif counter == 1:
					process_this_frame = True
				else:
					process_this_frame = False

				counter_search += 1

			try:
				if counter_search > 2000:
					counter_search = 0
					return "search"
					
				elif process_this_frame:
					face_locations = face_recognition.face_locations(img)
					face_encodings = face_recognition.face_encodings(img, face_locations)

					for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
						# See if the face is a match for the known face(s)
						matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

						name = "Unknown"

						face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
						best_match_index = np.argmin(face_distances)
						if matches[best_match_index]:
							name = known_face_names[best_match_index]
							for person in know_people.keys():
								if person == name:
									if know_people[person].role == "":
										know_people[person].role = "platform" +str(platform)
										#know_people[person].color = [(0, 0, 255),(0, 255, 0),(255, 0, 0)][platform-1]
										platform += 1
						
						center_frame = (int(mean([left, right])),int(mean([bottom,top])))
						font = cv.FONT_HERSHEY_DUPLEX

						if aim != None:
							if name in know_people.keys():
								if aim in roles:
									if (know_people[name].role == aim):
										return -(center_frame[0]-320)*(160/640)*2/3
								else:
									color = sizes[aim]
									if color in know_people[name].has_cube.keys():
										return -(center_frame[0]-320)*(160/640)*2/3
						else:
							if "green" not in know_people[name].has_cube.keys():
								try:
									green_cube = GreenCube(img, color='green', show_flag=True)
									if ((left - 40 < green_cube.center_x < right + 40) & (green_cube.center_y > bottom)):
										#know_people[name].has_cube.update({"green": green_cube})
										know_people[name].has_cube = {"green": green_cube}
										know_people[name].color = (0, 255, 0)
										for person in know_people.keys():
											if know_people[person].role == "platform3":
												know_people[person].role = know_people[name].role
												know_people[name].role = "platform3"
								except:
									pass
							if "orange" not in know_people[name].has_cube.keys():
								try:
									orange_cube = orangeCube(img, color='orange', show_flag=True)
									if ((left - 40 < orange_cube.center_x < right + 40) & (orange_cube.center_y > bottom)):
										#know_people[name].has_cube.update({"orange": orange_cube})
										know_people[name].has_cube = {"orange": orange_cube}
										know_people[name].color = (0, 0, 255)
								except:
									pass
							if "blue" not in know_people[name].has_cube.keys():
								try:
									blue_cube = BlueCube(img, color='blue', show_flag=True)
									if ((left - 40 < blue_cube.center_x < right + 40) & (blue_cube.center_y > bottom)):
										#know_people[name].has_cube.update({"blue": blue_cube})
										know_people[name].has_cube = {"blue": blue_cube}
										know_people[name].color = (255, 0, 0)
								except:
									pass

							
							if cubes >=3:
								cv.destroyAllWindows()
								finished = True
								return know_people

						if name in know_people.keys():
							color = know_people[name].color
							# Draw a box around the face
							cv.rectangle(img, (left, top), (right, bottom), color, 2)
							cv.circle(img, center_frame, 3, color, -1)

							# Draw a label with a name below the face
							cv.rectangle(img, (left, bottom - 35), (right, bottom), (255, 255, 255), cv.FILLED)
							cv.putText(img, name + know_people[name].role, (left + 6, bottom - 6), font, 1, color, 1)

							#giver_angle = (center_frame[0]-320)*(160/640)


			except:
				pass
			cv.imshow('human',img)
			cv.waitKey(1)
		except:
			pass


		cubes_dict = {}

		for person in know_people.keys():
			cubes_dict.update(know_people[person].has_cube)
			cubes = len(cubes_dict.keys())

		k=cv.waitKey(1)
		if k & 0xFF == ord('a'):
			cv.imwrite(str(i) + ".jpg", img)
			i=i+1

		if k & 0xFF == ord('q'):
			cv.destroyAllWindows()

# read one char (default)
def read_ser(num_char = 1):
	# reads return status from esp32

	string = port.read(num_char)
	return string.decode()

# Write whole strings
def write_ser(cmd):
	# Write command to esp32 via port

	cmd = cmd + '\n'
	port.write(cmd.encode())


if __name__ == '__main__':
	while(True):
  		brain()






