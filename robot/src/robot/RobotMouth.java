package robot;

import java.util.Locale;

import javax.speech.Central;
import javax.speech.EngineList;
import javax.speech.synthesis.Synthesizer;
import javax.speech.synthesis.SynthesizerModeDesc;
import javax.speech.synthesis.Voice;

public class RobotMouth {
	
	private Synthesizer synthesizer;
	
	RobotMouth() {
		try {
			SynthesizerModeDesc required = new SynthesizerModeDesc(null,"general",Locale.US,null,null);
			synthesizer = Central.createSynthesizer(required);
			synthesizer.allocate();
		} catch(Throwable t) {
			t.printStackTrace();
		}
	}

	void pause() {
		try {
			synthesizer.pause();
		} catch(Throwable t) {
			t.printStackTrace();
		}
	}
	
	void speak(String str) {
		try {
			synthesizer.resume();
			synthesizer.speakPlainText(str, null);
		} catch(Throwable t) {
			t.printStackTrace();
		}
	}
	
	void close() {
		try {
			synthesizer.deallocate();
		} catch(Throwable t) {
			t.printStackTrace();
		}
	}
}
