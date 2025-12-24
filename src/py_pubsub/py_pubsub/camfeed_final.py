from flask import Flask, Response, render_template_string
import cv2

app = Flask(__name__)

# Define all cameras here
camera_indices = [0, 2]  # Modify based on your setup
cameras = {i: cv2.VideoCapture(i) for i in camera_indices}

# Configure each camera for smoother streaming
for cam in cameras.values():
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)   # Lower resolution width
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)  # Lower resolution height
    cam.set(cv2.CAP_PROP_FPS, 20)            # Target ~20 FPS

def generate_frames(camera_id):
    cam = cameras[camera_id]
    while True:
        success, frame = cam.read()
        if not success:
            break
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 40]  # lower quality = smaller size
        ret, buffer = cv2.imencode('.jpg', frame, encode_param)
        frame = buffer.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    html = """
    <html>
    <head>
        <title>Multi-Camera Stream</title>
        <style>
            body { text-align: center; background: #111; color: #fff; font-family: Arial; }
            .cam-container { display: flex; justify-content: center; flex-wrap: wrap; }
            .cam-box { margin: 10px; background: #222; padding: 10px; border-radius: 10px; }
            img { width: 320px; border-radius: 8px; }
        </style>
    </head>
    <body>
        <h2>Low-Quality Multi-Camera Stream (Optimized for SSH)</h2>
        <div class="cam-container">
            {% for cam_id in cam_ids %}
                <div class="cam-box">
                    <h3>Camera {{ cam_id }}</h3>
                    <img src="/video/{{ cam_id }}">
                </div>
            {% endfor %}
        </div>
    </body>
    </html>
    """
    return render_template_string(html, cam_ids=camera_indices)

@app.route('/video/<int:camera_id>')
def video(camera_id):
    return Response(generate_frames(camera_id),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def main():
    print("Starting Flask multi-camera stream server...")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    main()
