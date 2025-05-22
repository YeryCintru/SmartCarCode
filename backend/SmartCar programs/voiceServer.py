from flask import Flask, request, jsonify
import threading
import os
from pydub import AudioSegment

class AudioServer:
    def __init__(self, smart_car, host='0.0.0.0', port=50007):
        self.app = Flask(__name__)
        self.smart_car = smart_car
        self.host = host
        self.port = port
        self.thread = None
        self.upload_folder = 'uploads'
        os.makedirs(self.upload_folder, exist_ok=True)

        # Definir rutas
        self.app.add_url_rule('/upload', 'upload_audio', self.upload_audio, methods=['POST'])
        self.app.add_url_rule('/shutdown', 'shutdown', self.shutdown, methods=['POST'])

    def upload_audio(self):
        if 'audio' not in request.files:
            return jsonify({'status': 'error', 'message': 'No se recibió archivo'}), 400

        file = request.files['audio']
        if file.filename == '':
            return jsonify({'status': 'error', 'message': 'Nombre de archivo vacío'}), 400

        original_path = os.path.join(self.upload_folder, file.filename)
        file.save(original_path)

        base_filename = os.path.splitext(file.filename)[0]
        wav_filename = f"{base_filename}.wav"
        wav_path = os.path.join(self.upload_folder, wav_filename)

        try:
            audio = AudioSegment.from_file(original_path)
            audio.export(wav_path, format="wav")
        except Exception as e:
            return jsonify({'status': 'error', 'message': f'Error al convertir audio: {str(e)}'}), 500

        # Aquí podrías llamar a smart_car.set_command(...) cuando tengas el comando listo

        return jsonify({
            'status': 'ok',
            'message': 'Audio recibido y convertido a WAV',
            'original': original_path,
            'converted': wav_path
        }), 200

    def shutdown(self):
        func = request.environ.get('werkzeug.server.shutdown')
        if func is None:
            raise RuntimeError('No running with the Werkzeug Server')
        func()
        return 'Server shutting down...'

    def start(self):
        if self.thread and self.thread.is_alive():
            print("Server already running")
            return

        def run():
            self.app.run(host=self.host, port=self.port)

        self.thread = threading.Thread(target=run)
        self.thread.start()
        print("Server started")

    def stop(self):
        import requests
        try:
            requests.post(f'http://{self.host}:{self.port}/shutdown')
            self.thread.join()
            print("Server stopped")
        except Exception as e:
            print("Error stopping server:", e)
