from scipy.ndimage import gaussian_filter1d
from numpy.random import default_rng
import matplotlib.pyplot as plt
from os import getcwd


###### NOISE PARAMETERS ######
NPOINTS = 200
SIGMA = 6

NOISE_INDEX = 1


###########################################
def generate_noise(npoints:int, sigma:int, plot:bool, plot_dir:str, noise_index:int):

    rng = default_rng()
    x = rng.normal(size=npoints) / 20.0
    y = gaussian_filter1d(x, sigma)
    print("Successfully generated %d noise datapoints!" % len(y))

    if plot:
        plt.plot(x, 'k', label='original noise')
        mylabel = "filtered noise, sigma=" + str(sigma)
        plt.plot(y, '--', label=mylabel)
        plt.legend()
        plt.grid()
        # plt.show()
        plt.savefig(plot_dir+'noise'+str(noise_index)+'.png')

    return x, y


###########################################
if __name__ == "__main__":

    noise_files_dir = getcwd()+'/robot_noise_files/'
    noise_file_name = 'noise'+str(NOISE_INDEX)+'.csv'
    plot_dir = noise_files_dir+'plots/'

    original, smoothed = generate_noise(NPOINTS, SIGMA, plot=True, plot_dir=plot_dir, noise_index=NOISE_INDEX)

    # save noise vector to csv file
    smoothed.tofile(noise_files_dir+noise_file_name, sep=',')