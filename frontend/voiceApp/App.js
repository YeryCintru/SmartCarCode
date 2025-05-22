import React, { useState } from 'react';
import { SafeAreaView, Button, Text, Alert, StyleSheet, View, TouchableOpacity } from 'react-native';
import { Audio } from 'expo-av';
import * as FileSystem from 'expo-file-system';

export default function App() {
  const [recording, setRecording] = useState(null);  // Estado para saber si estamos grabando
  const [audioUri, setAudioUri] = useState('');      // URI del archivo grabado

  // Función para comenzar a grabar
  const startRecording = async () => {
    try {
      // Pide permiso para grabar audio
      const permission = await Audio.requestPermissionsAsync();
      if (permission.granted) {
        // Inicia la grabación con una calidad alta
        const { recording } = await Audio.Recording.createAsync(
          Audio.RecordingOptionsPresets.HIGH_QUALITY
        );
        setRecording(recording);  // Guarda la grabación activa
        console.log('Recording...');
      } else {
        Alert.alert('Access denied', 'You need to enable microphone access to record audio.');
      }
    } catch (error) {
      console.error(error);
    }
  };

  // Función para detener la grabación
  const stopRecording = async () => {
    if (recording) {
      try {
        // Detiene la grabación
        await recording.stopAndUnloadAsync();
        const uri = recording.getURI();  // Obtiene la URI del archivo grabado
        setAudioUri(uri);  // Guarda la URI del archivo de audio
        setRecording(null);  // Limpia el estado de grabación
        console.log('Recording stoped', uri);
      } catch (error) {
        console.error('Error in stopping recording', error);
      }
    }
  };

  // Función para reproducir el audio grabado
  const playAudio = async () => {
    if (audioUri) {
      const { sound } = await Audio.Sound.createAsync(
        { uri: audioUri },
        { shouldPlay: true }
      );
      console.log('Playing audio...');
      await sound.playAsync();
    } else {
      Alert.alert('No audio recorded');
    }
  };

  // Función para enviar el archivo de audio al servidor
  const sendAudioToServer = async () => {
    try {
      // Si tenemos la URI del archivo grabado
      if (audioUri) {
        const formData = new FormData();
        const fileInfo = await FileSystem.getInfoAsync(audioUri);

        // Agrega el archivo al formData
        formData.append('audio', {
          uri: fileInfo.uri,
          name: 'audio.m4a',  // El nombre del archivo
          type: 'audio/mp4',  // Tipo de archivo
        });

        // Realiza el envío al servidor Python usando fetch
        const response = await fetch('http://192.168.98.47:50009/upload', {
          method: 'POST',
          body: formData,
          headers: {
            'Content-Type': 'multipart/form-data',
          },
        });

        // Verifica la respuesta del servidor
        const result = await response.json();
        console.log('Audio sended succesfully:', result);
      } else {
        Alert.alert('Error', 'No audio recorded to send');
      }
    } catch (error) {
      console.error('Error sending the audio:', error);
    }
  };

  return (
    <SafeAreaView style={styles.container}>
    <Text style={styles.title}>
      {!!recording ? 'Recording...' : 'Record your audio'}
    </Text>
    
    {/* Botones para grabar */}
    <View style={styles.buttonContainer}>
      <TouchableOpacity
        style={[styles.button, !!recording && styles.disabledButton]}
        onPress={startRecording}
        disabled={!!recording}
      >
        <Text style={styles.buttonText}>Start recording</Text>
      </TouchableOpacity>
      
      <TouchableOpacity
        style={[styles.button, !recording && styles.disabledButton]}
        onPress={stopRecording}
        disabled={!recording}
      >
        <Text style={styles.buttonText}>Stop recording</Text>
      </TouchableOpacity>
    </View>
  
    {/* Si tenemos una URI, mostrar los botones de reproducir y enviar */}
    {audioUri ? (
      <View style={styles.audioControls}>
        <TouchableOpacity
          style={styles.secondaryButton}
          onPress={playAudio}
        >
          <Text style={styles.buttonText}>Play audio</Text>
        </TouchableOpacity>
  
        <TouchableOpacity
          style={styles.secondaryButton}
          onPress={sendAudioToServer}
        >
          <Text style={styles.buttonText}>Send audio</Text>
        </TouchableOpacity>
      </View>
    ) : null}
  </SafeAreaView>
  
  );
}

// Estilos para la app
const styles = StyleSheet.create({
  container: {
    flex: 1,
    justifyContent: 'center',
    alignItems: 'center',
    backgroundColor: '#f0f0f0',
    padding: 20,
  },
  title: {
    fontSize: 24,
    fontWeight: 'bold',
    marginBottom: 30,
    color: '#333',
  },
  buttonContainer: {
    flexDirection: 'row',
    justifyContent: 'space-between',
    width: '100%',
    marginBottom: 20,
  },
  button: {
    backgroundColor: '#007bff',
    padding: 15,
    borderRadius: 8,
    width: '45%',
    alignItems: 'center',
    justifyContent: 'center',
  },
  disabledButton: {
    backgroundColor: '#dcdcdc',
  },
  secondaryButton: {
    backgroundColor: '#28a745',
    padding: 15,
    borderRadius: 8,
    marginBottom: 10,
    width: '80%',
    alignItems: 'center',
  },
  buttonText: {
    color: '#fff',
    fontSize: 16,
    fontWeight: 'bold',
  },
  audioControls: {
    marginTop: 30,
    width: '100%',
    alignItems: 'center',
  },
});
