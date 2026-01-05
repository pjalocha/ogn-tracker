# simple server to receive logs upload by OGN-Trackers
# author: GPT

from flask import Flask, request, jsonify
import os
import traceback

app = Flask(__name__)

UPLOAD_FOLDER = 'log_uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/upload', methods=['POST'])
def upload_file():
    try:
        # Get filename from header
        file_name = request.headers.get('X-File-Name', 'default_name.txt')
        file_path = os.path.join(UPLOAD_FOLDER, file_name)

        # Write streamed data to file
        with open(file_path, 'wb') as f:
            while True:
                chunk = request.stream.read(1024)  # Read in chunks of 1KB
                if not chunk:
                    break
                f.write(chunk)

        return jsonify({"message": f"File {file_name} uploaded successfully"}), 200

    except Exception as e:
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8084)
