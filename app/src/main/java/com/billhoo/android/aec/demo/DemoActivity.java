package com.billhoo.android.aec.demo;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.Field;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import android.app.Activity;
import android.app.AlertDialog;
import android.media.AudioFormat;
import android.media.AudioManager;
import android.media.AudioRecord;
import android.media.AudioTrack;
import android.media.MediaRecorder;
import android.net.Uri;
import android.os.Bundle;
import android.os.Environment;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import com.android.webrtc.audio.MobileAEC;
import com.android.webrtc.audio.R;
import android.os.Process;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.TextView;

/**
 * This demo will show you how to use MobileAEC class to deal with echo things
 * on android.<br>
 * </ul> <br>
 * <br>
 * <b>[NOTICE]</b>
 * <ul>
 * you should add <b>"android.permission.WRITE_EXTERNAL_STORAGE"</b> in your
 * AndroidManifest.xml file to allow this DEMO write data into your SDcard.
 * </ul>
 * <b>[TODO List]</b>
 * <ul>
 * <li>
 * (billhoo): should write all the demo processes into separate thread instead
 * of the UI thread.</li>
 * </ul>
 *
 * @author billhoo E-mail: billhoo@126.com
 */
public class DemoActivity extends Activity {
	private static final boolean AECM_DEBUG = true;
	/**
	 * ATTENTION: This was auto-generated to implement the App Indexing API.
	 * See https://g.co/AppIndexing/AndroidStudio for more information.
	 */

	private RecordThread _recordThread;
	private TrackThread  _trackThread;

	private final int PORT = 13000;

	DatagramSocket _dataSocket;

	MobileAEC _aecm;

	InetAddress _dstAddress;

	EditText _editTextIp;
	CheckBox _checkBoxAec;
	EditText _editTextDelay;
	Button _btn;

	short _bufferDelay = 150; //ms

	MobileAEC.AggressiveMode _mode = MobileAEC.AggressiveMode.AGGRESSIVE;


	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);


		_editTextIp = (EditText)findViewById(R.id.editTarget);
		_checkBoxAec = (CheckBox)findViewById(R.id.checkBoxAEC);
		_editTextDelay = (EditText)findViewById(R.id.editBufferDelay);

		_editTextDelay.addTextChangedListener(new TextWatcher() {
			@Override
			public void beforeTextChanged(CharSequence s, int start, int count, int after) {

			}

			@Override
			public void onTextChanged(CharSequence s, int start, int before, int count) {

			}

			@Override
			public void afterTextChanged(Editable s) {
				try {
					_bufferDelay = (short) Integer.parseInt(s.toString());
				}catch (Exception ex){
					_bufferDelay = 0;
				}
			}
		});


		RadioGroup group = (RadioGroup)findViewById(R.id.radioGroupOpt);
		group.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
			@Override
			public void onCheckedChanged(RadioGroup group, int checkedId) {
				if(checkedId == R.id.radioButton0){
					_mode = MobileAEC.AggressiveMode.MILD;
				}else if(checkedId == R.id.radioButton1){
					_mode = MobileAEC.AggressiveMode.MEDIUM;
				}else if(checkedId == R.id.radioButton2){
					_mode = MobileAEC.AggressiveMode.HIGH;
				}else if(checkedId == R.id.radioButton3){
					_mode = MobileAEC.AggressiveMode.AGGRESSIVE;
				}else if(checkedId == R.id.radioButton4){
					_mode = MobileAEC.AggressiveMode.MOST_AGGRESSIVE;
				}
			}
		});

		_btn = (Button) findViewById(R.id.btnStart);

		_btn.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View v) {
				if(_trackThread == null || _recordThread == null) {

					try {
						_dstAddress = InetAddress.getByName(_editTextIp.getText().toString());
						_dataSocket = new DatagramSocket(PORT);
					}catch (SocketException ex){
						new AlertDialog.Builder(DemoActivity.this).setTitle("系统提示")
						     .setMessage(ex.getMessage())
						     .show();
						return ;
					}catch (UnknownHostException ex){
						new AlertDialog.Builder(DemoActivity.this).setTitle("系统提示")
								.setMessage(ex.getMessage())
								.show();
						return ;
					}catch (Exception ex){
						new AlertDialog.Builder(DemoActivity.this).setTitle("系统提示")
								.setMessage("网络错误"+ex.getMessage())
								.show();
						return ;
					}

					try{
						_aecm = new MobileAEC(null);
						_aecm.setAecmMode(_mode)
								.prepare();

					}catch(Exception ex){
						new AlertDialog.Builder(DemoActivity.this).setTitle("系统提示")
								.setMessage(ex.getMessage())
								.show();
						return ;
					}

					_trackThread = new TrackThread();
					_trackThread.Start();

					_recordThread = new RecordThread();
					_recordThread.Start();

					UpdateUI(true);

				}else{
					_dataSocket.close();
					_recordThread.Stop();
					_trackThread.Stop();
					_recordThread = null;
					_trackThread = null;
					_aecm.close();
					UpdateUI(false);
				}
			}
		});
	}

	private void UpdateUI(boolean toStart){
		RadioButton[] r = new RadioButton[5];
		r[0] = (RadioButton)findViewById(R.id.radioButton0);
		r[1] = (RadioButton)findViewById(R.id.radioButton1);
		r[2] = (RadioButton)findViewById(R.id.radioButton2);
		r[3] = (RadioButton)findViewById(R.id.radioButton3);
		r[4] = (RadioButton)findViewById(R.id.radioButton4);

		String str;
		if(toStart) {
			str = "Stop";
		}else{
			str = "Start";
		}

		_btn.setText(str);
		_editTextIp.setEnabled(!toStart);
		_checkBoxAec.setEnabled(!toStart);
		for(int i = 0 ; i < 5; ++i){
			r[i].setEnabled(!toStart);
		}

	}


	private class RecordThread extends Thread{

		private boolean _done = false;
		private AudioRecord _audioRecord = null;

		private static final int CHANNELS = 1;

		// Default audio data format is PCM 16 bit per sample.
		// Guaranteed to be supported by all devices.
		private static final int BITS_PER_SAMPLE = 16;

		private static final int SAMPLE_RATE = 16000;

		// Requested size of each recorded buffer provided to the client.
		private static final int CALLBACK_BUFFER_SIZE_MS = 10;

		// Average number of callbacks per second.
		private static final int BUFFERS_PER_SECOND = 1000 / CALLBACK_BUFFER_SIZE_MS;

		// We ask for a native buffer size of BUFFER_SIZE_FACTOR * (minimum required
		// buffer size). The extra space is allocated to guard against glitches under
		// high load.
		private static final int BUFFER_SIZE_FACTOR = 2;

		public void Start(){
			_done = false;
			this.start();
		}
		public void Stop(){
			_done = true;
			try {
				this.join();
			}catch (InterruptedException ex){

			}
		}

		@Override
		public void run() {
			super.run();

			Process.setThreadPriority(Process.THREAD_PRIORITY_URGENT_AUDIO);

			final int bytesPerFrame = CHANNELS * (BITS_PER_SAMPLE / 8);
			final int framesPerBuffer = SAMPLE_RATE / BUFFERS_PER_SECOND;
			ByteBuffer byteBuffer = ByteBuffer.allocateDirect(bytesPerFrame * framesPerBuffer);

			int minBufferSize = AudioRecord.getMinBufferSize(
					SAMPLE_RATE,
					AudioFormat.CHANNEL_IN_MONO,
					AudioFormat.ENCODING_PCM_16BIT);

			// Use a larger buffer size than the minimum required when creating the
			// AudioRecord instance to ensure smooth recording under load. It has been
			// verified that it does not increase the actual recording latency.

			int bufferSizeInBytes =
					Math.max(BUFFER_SIZE_FACTOR * minBufferSize, 0);

			short[] aecTmpOut = new short[byteBuffer.capacity() / 2];
			short[] aecTmpIn = new short[byteBuffer.capacity() / 2];
			byte[] sendBuffer = new byte[byteBuffer.capacity()];

			DatagramPacket dataPacket;
			try {
				if(_checkBoxAec.isChecked()) {
					dataPacket = new DatagramPacket(sendBuffer, 0, sendBuffer.length, _dstAddress, PORT);
				}else
				{
					dataPacket = new DatagramPacket(byteBuffer.array(), byteBuffer.arrayOffset(), byteBuffer.capacity(), _dstAddress, PORT);
				}

				_audioRecord = new AudioRecord(MediaRecorder.AudioSource.VOICE_COMMUNICATION,
						SAMPLE_RATE,
						AudioFormat.CHANNEL_IN_MONO,
						AudioFormat.ENCODING_PCM_16BIT,
						bufferSizeInBytes);

				_audioRecord.startRecording();

			} catch (IllegalArgumentException ex) {
				Log.e("record", ex.getMessage());
				return;
			}


			try {
				while (!_done) {

					int bytesRead = _audioRecord.read(byteBuffer, byteBuffer.capacity());
					if (bytesRead == byteBuffer.capacity()) {

						if(_checkBoxAec.isChecked()) {
							ByteBuffer.wrap(byteBuffer.array()).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(aecTmpIn);

							_aecm.echoCancellation(aecTmpIn, null, aecTmpOut,
									(short) (byteBuffer.capacity() / 2), _bufferDelay);


							ByteBuffer.wrap(sendBuffer).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().put(aecTmpOut);
						}

						_dataSocket.send(dataPacket);

					} else {
						Log.e("record", "AudioRecord.read failed: " + bytesRead);
					}
				}
			} catch (IOException ex) {
				Log.e("record", ex.getMessage());
			}catch (Exception ex) {
				Log.e("record", ex.getMessage());
			}
			_audioRecord.stop();
			_audioRecord.release();
		}
	}

	private class TrackThread extends Thread{


		private static final int CHANNELS = 1;
		// Default audio data format is PCM 16 bit per sample.
		// Guaranteed to be supported by all devices.
		private static final int BITS_PER_SAMPLE = 16;

		private static final int SAMPLE_RATE = 16000;

		// Requested size of each recorded buffer provided to the client.
		private static final int CALLBACK_BUFFER_SIZE_MS = 10;

		// Average number of callbacks per second.
		private static final int BUFFERS_PER_SECOND = 1000 / CALLBACK_BUFFER_SIZE_MS;

		private boolean _done = false;

		private AudioTrack _audioTrack = null;

		public void Start(){
			_done = false;
			this.start();
		}
		public void Stop(){
			_done = true;
			try {
				this.join();
			}catch (InterruptedException ex){

			}
		}

		@Override
		public void run() {
			super.run();

			Process.setThreadPriority(Process.THREAD_PRIORITY_URGENT_AUDIO);

			final int bytesPerFrame = CHANNELS * (BITS_PER_SAMPLE / 8);
			ByteBuffer byteBuffer = ByteBuffer.allocateDirect(
					bytesPerFrame * (SAMPLE_RATE / BUFFERS_PER_SECOND));

			// Get the minimum buffer size required for the successful creation of an
			// AudioTrack object to be created in the MODE_STREAM mode.
			// Note that this size doesn't guarantee a smooth playback under load.
			// TODO(henrika): should we extend the buffer size to avoid glitches?
			final int minBufferSizeInBytes = AudioTrack.getMinBufferSize(
					SAMPLE_RATE,
					AudioFormat.CHANNEL_OUT_MONO,
					AudioFormat.ENCODING_PCM_16BIT);

			DatagramPacket dataPacket = new DatagramPacket(byteBuffer.array(), byteBuffer.arrayOffset(), byteBuffer.capacity());
			try {

				// Create an AudioTrack object and initialize its associated audio buffer.
				// The size of this buffer determines how long an AudioTrack can play
				// before running out of data.
				_audioTrack = new AudioTrack(AudioManager.STREAM_VOICE_CALL,
						SAMPLE_RATE,
						AudioFormat.CHANNEL_OUT_MONO,
						AudioFormat.ENCODING_PCM_16BIT,
						minBufferSizeInBytes,
						AudioTrack.MODE_STREAM);

				_audioTrack.play();

			} catch (IllegalArgumentException e) {
				Log.e("track", e.getMessage());
				return;
			}

			final int sizeInBytes = byteBuffer.capacity();
			short[] aecTmpIn = new short[sizeInBytes / 2];

			try {
				while (!_done) {

					_dataSocket.receive(dataPacket);

					int bytesWritten = _audioTrack.write(byteBuffer.array(), byteBuffer.arrayOffset(), sizeInBytes);
					if (bytesWritten == sizeInBytes){
						if(_checkBoxAec.isChecked()) {
							ByteBuffer.wrap(byteBuffer.array()).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(aecTmpIn);
							_aecm.farendBuffer(aecTmpIn, sizeInBytes / 2);
						}

					}else{
						_done = true;
					}
				}
			}catch (IOException ex){
				Log.e("track", ex.getMessage());
			}catch (Exception ex){
				Log.e("track", ex.getMessage());
			}

			_audioTrack.stop();
			_audioTrack.release();
		}
	}

	// //////////////////////////////////////////////////////////////////////////
	// ACOUSTIC ECHO CANCELLATION MOBILE EDITION

	public void doAECM() {
		try {
			MobileAEC aecm = new MobileAEC(null);
			aecm.setAecmMode(MobileAEC.AggressiveMode.MOST_AGGRESSIVE)
					.prepare();

			// get pcm raw data file in root of android sd card.
			// if you test this demo, you should put these files into your
			// android device or emulator.
			// the ideal output of pcm is almost down to zero.
			FileInputStream fin = new FileInputStream(new File(
					Environment.getExternalStorageDirectory()
							+ "/Android/en-00-raw-pcm-16000Hz-16bit-mono.pcm"));

			FileOutputStream fout = new FileOutputStream(new File(
					Environment.getExternalStorageDirectory()
							+ "/Android/aecm.pcm"));

			final int cacheSize = 320;
			byte[] pcmInputCache = new byte[cacheSize];

			// core procession
			for (/* empty */; fin.read(pcmInputCache, 0, pcmInputCache.length) != -1; /* empty */) {
				// convert bytes[] to shorts[], and make it into little endian.
				short[] aecTmpIn = new short[cacheSize / 2];
				short[] aecTmpOut = new short[cacheSize / 2];
				ByteBuffer.wrap(pcmInputCache).order(ByteOrder.LITTLE_ENDIAN).asShortBuffer().get(aecTmpIn);

				// aecm procession, for now the echo tail is hard-coded 10ms,
				// but you
				// should estimate it correctly each time you call
				// echoCancellation, otherwise aecm
				// cannot work.
				aecm.farendBuffer(aecTmpIn, cacheSize / 2);
				aecm.echoCancellation(aecTmpIn, null, aecTmpOut,
						(short) (cacheSize / 2), (short) 10);

				// output
				byte[] aecBuf = new byte[cacheSize];
				ByteBuffer.wrap(aecBuf).order(ByteOrder.LITTLE_ENDIAN)
						.asShortBuffer().put(aecTmpOut);

				fout.write(aecBuf);
			}

			fout.close();
			fin.close();
			aecm.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


}