import cv2
import os

command = 'sudo modprobe bcm2835-v4l2'

os.system(command)

# Open video capture
cap = cv2.VideoCapture(0)

# Define detector
detector = cv2.QRCodeDetector()

while True:

	ret, frame = cap.read()

	data, bbox, _ = detector.detectAndDecode(frame)

	if(bbox is not None):
		for i in range(len(bbox)):
			cv2.line(frame, tuple(bbox[i][0]), tuple(bbox[(i + 1) % len(bbox)][0]), color= (90, 0, 255), thickness= 4)
			cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

	if data:
		print(f'Data: {data}')

	# Show result to the screen
	cv2.imshow('QrCode Detector', frame)

	# Break the loop by pressing q key
	if(cv2.waitKey(1) == ord('q')):
		break

cap.release()
cv2.destroyAllWindows()

