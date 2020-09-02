# import the opencv library 
import cv2 
import sys  

def list_supported_capture_properties(cap: cv2.VideoCapture):
    """ List the properties supported by the capture device.
    """
    supported = list()
    for attr in dir(cv2):
        if attr.startswith('CAP_PROP'):
            if cap.get(getattr(cv2, attr)) != -1:
                supported.append(attr)
    return supported
  
# define a video capture object 
camerafile = sys.argv[1]
vid = cv2.VideoCapture(camerafile) 
list_supported_capture_properties(vid)
  
while(True): 
      
    # Capture the video frame 
    # by frame 
    ret, frame = vid.read() 
  
    # Display the resulting frame 
    cv2.imshow('frame', frame) 
      
    # the 'q' button is set as the 
    # quitting button you may use any 
    # desired button of your choice 
    if cv2.waitKey(1) & 0xFF == ord('q'): 
        break
  
# After the loop release the cap object 
vid.release() 
# Destroy all the windows 
cv2.destroyAllWindows() 

