clear all;
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%
random = false; % Use random initial position and velocity?
radar_max_range = 200; % meters
radar_max_velocity = 100; % meters/sec
radar_range_resolution = 1; % meters

%speed of light = 3e8
c = 3e8; % speed of light in meters/sec

%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
target_position = 110; % meters
target_velocity = -20; % meters/sec

if (random)
    target_position = randi(200); % meters
    target_velocity = randi([-70, 70]); % meters/sec
end

fprintf("Initial position: %d and velocity: %d\n", target_position, target_velocity);
%% FMCW Waveform Generation

% *%TODO* :
%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

B = c / (2 * radar_range_resolution);
Tchirp = 5.5 * 2 * radar_max_range / c;
slope = B / Tchirp;  % alpha
fprintf("Bandwidth: %d and Chirp Time: %d\n", B, Tchirp);
fprintf("Slope: %d\n", slope);

%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq
                                
%The number of chirps in one sequence. Its ideal to have 2^ value for the ease of running the FFT
%for Doppler Estimation. 
Nd=128;                   % #of doppler cells OR #of sent periods % number of chirps

%The number of samples on each chirp. 
Nr=1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t=linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples

%Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx=zeros(1,length(t)); %transmitted signal
Rx=zeros(1,length(t)); %received signal
Mix = zeros(1,length(t)); %beat signal

%Similar vectors for range_covered and time delay.
r_t=zeros(1,length(t)); % vector of range values
td=zeros(1,length(t)); % vector of tau


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity.
    % Constant velocity model: Xnew = Xprev + v * t
    r_t(i) = target_position + target_velocity * t(i);
    td(i) = 2 * r_t(i) / c; % Range = (c / 2) * tau
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    Tx(i) = cos(2 * pi * (fc * t(i) + slope * (t(i)^2)/2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + slope * ((t(i) - td(i))^2)/2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i).*Rx(i);
    
end

%% RANGE MEASUREMENT

 % *%TODO* :
%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Y = reshape(Mix, Nr, Nd); % size: 1024 x 128

 % *%TODO* :
%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
Y = fft(Y, Nr);
Y = Y./Nr;

 % *%TODO* :
% Take the absolute value of FFT output
Y = abs(Y);

 % *%TODO* :
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
Y = Y(1:Nr/2);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

 % *%TODO* :
 % plot FFT output 
plot(Y);
 
axis ([0 200 0 1]);



%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

Mix=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

% *%TODO* :
%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 8;

% *%TODO* :
%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 4;
Gd = 4;

% *%TODO* :
% offset the threshold by SNR value in dB
offset = 8; % Used 5, 6, 7 but 8 and above gives better result

% *%TODO* :
%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);  % Seems unnecessary to initialize here ...


% *%TODO* :
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.

   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   
% Calculate number of training cells for calculating average later   
num_training_cells = (2*Tr+2*Gr+1)*(2*Td+2*Gd+1)-(2*Gr+1)*(2*Gd+1);
fprintf("# of training cells: %d\n", num_training_cells);

% Index in position 1 exceeds array bounds (must not exceed 512).
[M,N] = size(RDM);
fprintf("Size of RDM matrix: %d x %d\n", M, N);

for i = Tr+Gr+1:M-(Tr+Gr)
    for j = Td+Gd+1:Nd-(Td+Gd)
        
        for p = i-(Tr+Gr):i+(Tr+Gr)
            for q = j-(Td+Gd):j+(Td+Gd)
                
                % check if p & q within training cells indices, if they are
                % then add RDM[p,q] to noise_level
                if abs(i-p) || abs(j-q) 
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
        % Average the summed values for all of the training cells used
        avg_noise_level = noise_level / num_training_cells;
        
        % After averaging convert it back to logarithimic using pow2db
        avg_noise_level_db = pow2db(avg_noise_level);
        
        % Add the offset to it to determine the threshold
        threshold = avg_noise_level_db + offset;
        
        % Signal under CUT
        CUT = RDM(i,j);
        
        % If the CUT level > threshold assign it a value of 1, else equate it to 0
        if CUT > threshold
            RDM(i,j) = 1;
        else
            RDM(i,j) = 0;
        end
        
        % Reinitialize noise_level vector with zeros for next CUT
        noise_level = zeros(1,1);
    end
end

fprintf("\nCell Averaging CFAR finished!\n");

% *%TODO* :
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 

for i = 1:M
    for j = 1:Nd
        % Outer cells of RDM matrix has values other than 0 or 1 so set
        % them to 0
        if RDM(i,j) ~= 0 && RDM(i,j) ~= 1
            RDM(i,j) = 0;
        end
    end
end


% *%TODO* :
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis,range_axis,RDM);
colorbar;
