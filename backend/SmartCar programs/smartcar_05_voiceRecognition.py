from flask import Flask, request, jsonify
from pydub import AudioSegment
import smartcar
import os

app = Flask(__name__)
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/upload', methods=['POST'])
def upload_audio():
    sc = smartcar.SmartCar()
    if 'audio' not in request.files:
        return jsonify({'status': 'error', 'message': 'No se recibió archivo'}), 400

    file = request.files['audio']
    if file.filename == '':
        return jsonify({'status': 'error', 'message': 'Nombre de archivo vacío'}), 400

    # Guarda el archivo original
    original_path = os.path.join(UPLOAD_FOLDER, file.filename)
    file.save(original_path)

    # Determina nombre de salida .wav
    base_filename = os.path.splitext(file.filename)[0]
    wav_filename = f"{base_filename}.wav"
    wav_path = os.path.join(UPLOAD_FOLDER, wav_filename)

    try:
        # Convertir a .wav con pydub
        audio = AudioSegment.from_file(original_path)
        audio.export(wav_path, format="wav")
    except Exception as e:
        return jsonify({'status': 'error', 'message': f'Error al convertir audio: {str(e)}'}), 500

    return jsonify({
        'status': 'ok',
        'message': 'Audio recibido y convertido a WAV',
        'original': original_path,
        'converted': wav_path
    }), 200

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=50007)
