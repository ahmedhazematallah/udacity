clear all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

%speed of light = 3e8
c = 3e8;

%% User Defined Range and Velocity of target
% *%TODO* :
% define the target's initial position and velocity. Note : Velocity
% remains contant
 
R = 120;    % Initial distance to the target (Max 200)
v = 30;    % Initial speed of the target (from -70 to +70)


%% FMCW Waveform Generation


%Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.


%Operating carrier frequency of Radar 
fc= 77e9;             %carrier freq

% Range Resolution = 1m 
% d_Res = c / (2B)
d_res = 1;

% Maximum Range
R_max = 200;

% bandwidth B
% Result = 150 MHz
B = c / (2 * d_res);

% Calculate the time of the chirp
Tchirp = 5.5 * 2 * R_max / c;

% Calculate the slope
slope = B / Tchirp;


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
r_t=zeros(1,length(t));
td=zeros(1,length(t));


%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    
    % *%TODO* :
    %For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R + v * t(i);
    
    td(i) = r_t(i) / c * 2;
    
    % *%TODO* :
    %For each time sample we need update the transmitted and
    %received signal. 
    % Tx=cos(2π(f * t + slope t^2 / 2))

    Tx(i) = cos(2 * pi * (fc * t(i) + slope * (t(i) ^2) / 2));
    
    Rx(i) = cos(2 * pi * (fc * (t(i)-td(i)) + slope * ((t(i) - td(i))^2) / 2));
    
    % *%TODO* :
    %Now by mixing the Transmit and Receive generate the beat signal
    %This is done by element wise matrix multiplication of Transmit and
    %Receiver Signal
    Mix(i) = Tx(i) * Rx(i);
    
end

%% RANGE MEASUREMENT

%reshape the vector into Nr*Nd array. Nr and Nd here would also define the size of
%Range and Doppler FFT respectively.
Mix_reshaped = reshape(Mix, [Nr, Nd]);

%run the FFT on the beat signal along the range bins dimension (Nr) and
%normalize.
Mix_reshaped_fft = fft(Mix_reshaped, [], 1);

% Take the absolute value of FFT output
% Normalize the value too
Mix_reshaped_fft_abs = abs(Mix_reshaped_fft / Nr);
 
% Output of FFT is double sided signal, but we are interested in only one side of the spectrum.
% Hence we throw out half of the samples.
Mix_reshaped_fft_abs_single = 2* Mix_reshaped_fft_abs(1:Nr/2,:);

%plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)

% plot FFT output 

% Calculate the range axis
my_range_axis = d_res * (0:(Nr/2)-1);

% for the y-axis, select any vector of the Nd vectors, i choose the first
plot(my_range_axis, Mix_reshaped_fft_abs_single(:,1));
 
axis ([0 200 0 1]);


%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM


% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.

sig_resh=reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(sig_resh,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM) ;

%use the surf function to plot the output of 2DFFT and to show axis in both
%dimensions
doppler_axis = linspace(-100,100,Nd); % velocity -70 to +70
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400); % Range 0-200
figure,surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

%Slide Window through the complete Range Doppler Map

%Select the number of Training Cells in both the dimensions.
Tr = 10;
Td = 12;

%Select the number of Guard Cells in both dimensions around the Cell under 
%test (CUT) for accurate estimation
Gr = 3;
Gd = 6;

% offset the threshold by SNR value in dB
offset = 6;

%Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(Nr/2 - (2*Tr + 2*Gr + 1), ...
                    Nd - (2*Td + 2*Gd + 1));

% Number of training cells
numOfTrainingCells = (2*Tr+2*Gr+1) * (2*Td+2*Gd+1) - (2*Gr+1)*(2*Gd+1);

%Create a vector for the filtered signal after checking it vs the threshold
CFAR_result = zeros(Nr/2, Nd);
                
%design a loop such that it slides the CUT across range doppler map by
%giving margins at the edges for Training and Guard Cells.
%For every iteration sum the signal level within all the training
%cells. To sum convert the value from logarithmic to linear using db2pow
%function. Average the summed values for all of the training
%cells used. After averaging convert it back to logarithimic using pow2db.
%Further add the offset to it to determine the threshold. Next, compare the
%signal under CUT with this threshold. If the CUT level > threshold assign
%it a value of 1, else equate it to 0.
for row = 1:Nr/2 - (2*Tr + 2*Gr)
    for col = 1:Nd - (2*Td + 2*Gd)

   % Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
   % CFAR
   
   % Loop on the grid and add each element that belongs to the training
   % cells
        % Reset the noise value
        noise = 0;
        
        % Calculate the sum of the noise of all the training cells
        for i = row:row + (2*Tr + 2*Gr)
            for j = col:col + (2*Td + 2*Gd)
                   
                % Skip the guard cells & CUT
                if ( (i >= (row + Tr)) && (i <= (row + Tr + 2 * Gr)) && ...
                     (j >= (col + Td)) && (j <= (col + Td + 2 * Gd)) )
                    continue
                end
            
                % Add the noise of the current training cell
                noise = noise + db2pow(RDM(i,j));
            end
        end
        
        % Divide the sum by the number of training cells to obtain the 
        % average                    
        noise_level(row,col) = noise / numOfTrainingCells;
        
        % Convert the noise back to dB and add the offset
        noise_level(row,col) = pow2db(noise_level(row,col)) + offset;
        
        % Obtain the value of the CUT
        cut_x = row + Tr + Gr;
        cut_y = col + Td + Gd;
        cut = RDM(cut_x, cut_y);
        
        % Compare it to the noise level
        if (cut < noise_level(row,col))
            CFAR_result(cut_x, cut_y) = 0;
        else
            CFAR_result(cut_x, cut_y) = 1;
        end
    end
end
            
% The process above will generate a thresholded block, which is smaller 
%than the Range Doppler Map as the CUT cannot be located at the edges of
%matrix. Hence,few cells will not be thresholded. To keep the map size same
% set those values to 0. 
 
%display the CFAR output using the Surf function like we did for Range
%Doppler Response output.
figure,surf(doppler_axis, range_axis, CFAR_result);
colorbar;


% 
% Measure and average the noise across all the training cells. 
% This gives the threshold
% 
% Add the offset (if in signal strength in dB) to the threshold to keep 
% the false alarm to the minimum.
% 
% Determine the signal level at the Cell Under Test.
% 
% If the CUT signal level is greater than the Threshold, assign a value 
% of 1, else equate it to zero.
% 
% Since the cell under test are not located at the edges, due to the 
% training cells occupying the edges, we suppress the edges to zero. 
% Any cell value that is neither 1 nor a 0, assign it a zero.


 