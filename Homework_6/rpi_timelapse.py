
import cv2

def main():

    video_title = 'teleop.mp4'
    cap = cv2.VideoCapture(0)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(video_title, fourcc, 20.0, (640, 480))

    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 0)

        if ret:    # frame captured without any errors
            cv2.imshow("Feed", frame)

            # output the frame
            out.write(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    cap.release()

    out.release()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
