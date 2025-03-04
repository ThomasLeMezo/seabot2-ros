
import numpy as np

class ChirpContainer:
    def __init__(self, start_freq, end_freq, duration, fade_time=2e-3, sample_rate=192e3, extra_time=0):
        self.start_freq = start_freq
        self.end_freq = end_freq
        self.duration = duration
        self.fade_time = fade_time
        self.sample_rate = sample_rate
        self.extra_time = extra_time

        self.samples = None
        self.is_seabot = False

        self.seabot_dac_interp_p2 = -9.08900731e-08
        self.seabot_dac_interp_p1 = 5.70147051e-04
        self.seabot_dac_interp_p0 = 9.35330181e+02
        self.seabot_dac_polyfunc = np.poly1d([self.seabot_dac_interp_p2, self.seabot_dac_interp_p1, self.seabot_dac_interp_p0])

    def generate_chirp(self):
        t = np.linspace(0, self.duration, int(self.duration * self.sample_rate))
        time_ratio = t / self.duration
        frequency = self.start_freq + (self.end_freq - self.start_freq) * (time_ratio - 0.5)
        # Set the amplitude
        dac_amplitude = None
        if self.is_seabot:
            dac_amplitude = self.seabot_dac_polyfunc(frequency)
            dac_amplitude /= np.max(dac_amplitude)
        else:
            dac_amplitude = 1.0
        # Add fade in and fade out
        dac_amplitude = dac_amplitude * np.minimum(1.0, t / self.fade_time) * np.minimum(1.0, np.maximum(0.0, (self.duration - t) / self.fade_time))

        # Generate the chirp
        self.samples = dac_amplitude * np.sin(2.0 * np.pi * t * (self.start_freq + (self.end_freq - self.start_freq) * (time_ratio / 2.0 - 0.5)))

    def plot_chirp(self):
        import matplotlib.pyplot as plt
        # Plot temporal signal and frequency signal (after fft)
        t = np.linspace(0, self.duration, int(self.duration * self.sample_rate))
        plt.subplot(2, 1, 1)
        plt.plot(t, self.samples)
        plt.title('Chirp signal')
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.grid()
        # plot frequency function of time 2D mesh
        plt.subplot(2, 1, 2)
        plt.specgram(self.samples, Fs=self.sample_rate, NFFT=512, noverlap=256)
        plt.title('Frequency content')
        plt.xlabel('Time (s)')
        plt.ylabel('Frequency (Hz)')
        plt.grid()
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    # Create a chirp container
    chirp = ChirpContainer(38500, 42500, 0.05, 2e-3, 192e3)
    # Generate the chirp
    chirp.generate_chirp()
    # Plot the chirp
    chirp.plot_chirp()
