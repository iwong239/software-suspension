%% Lunasim Offloader Animator
% This function offers a basic animator that will take in an Active struct
% created by the Controls_Workspace script, and animate it. This will be
% used for creating live video for CDR and other presentations, as well as
% allowing us to demo the operator UI.

function AnimateLunaSim(Active, user_h)
% Outlining Time Frames. Change FramesPerSecond to change the video.
FramesPerSecond = 20;
frames = floor((FramesPerSecond * Active.T(end)));
time = linspace(0, Active.T(end), frames);


% Interpolating positions at timesteps.
    for i = 1:frames
        index = find((Active.T <= time(i)), 1, 'last');
        X(i) = interp1(Active.T, Active.X, time(i));
        Y(i) = interp1(Active.T, Active.Y, time(i));
        Z(i) = interp1(Active.T, Active.Z, time(i));
    end
    
    
% Plotting and creating movie from above
    %Creating Figure @ position to match resolutions
    ani = figure;
    set(ani, 'Position' ,[200 100 1060 1100]);
    
    for i = 1:frames
       clf;
       Box = GetUserCoords(X(i), Y(i), Z(i), user_h); 
       if (i <= 4)
            n = i-1;
       else
            n = 4;
       end
       %Plotting, as per usual
       plot3(X(i), Y(i), Z(i), 'o', 'LineWidth', 1.5);
       hold ('on');
       plot3(X(i-n:i), Y(i-n:i), Z(i-n:i), 'LineWidth', 1.5);
       plot3(Box(:,1), Box(:,2), Box(:,3), 'LineWidth', 1.5);
       hold ('off');
       %Setting Axes aesthetic stuff
       title('LunaSim User Space');
       axis ('equal'); grid on; 
       ax1 = gca;
       ax1.GridAlpha = 0.4;
       axis([0, 9*12, 0, 10*12, 0, 9*12]);
       
       %Capturing figure as a movie frame, and turning it into a movie
       MovieVector(i) = getframe(ani);
       
       % Progress bar for testing
%        progress = time(i)/time(end) * 100;
%        fprintf("   %.2f%% Complete\n", progress);

    end

    % For testing: checking if we save the animation
    %check = input("\nSave Animation?   [Y/N]:\n", 's');
    
    
    %if ((check == 'Y')||(check == 'y'))
        myWriter = VideoWriter('JumpAnimation');
        myWriter.FrameRate = FramesPerSecond;

        open(myWriter);
        writeVideo(myWriter, MovieVector);
        close(myWriter);
    %end

    close(ani);
end



function BoxCoords = GetUserCoords(x,y,z,user_h)
% Function same as in main script. Returns coordinates to plot a box the
% approximate size of the user.
    x1 = x - 20/2;
    x2 = x + 20/2;
    y1 = y - 40/2;
    y2 = y + 40/2;
    z1 = z - user_h/2;
    z2 = z + user_h/2;

    BoxCoords = [x1 y1 z1; ...
                 x1 y2 z1; ...
                 x2 y2 z1; ...
                 x2 y1 z1; ...
                 x1 y1 z1; ...
                 x1 y1 z2; ...
                 x1 y2 z2; ...
                 x1 y2 z1; ...
                 x1 y2 z2; ...
                 x2 y2 z2; ...
                 x2 y2 z1; ...
                 x2 y2 z2; ...
                 x2 y1 z2; ...
                 x2 y1 z1; ...
                 x2 y1 z2; ...
                 x1 y1 z2];
end