function [] = plotData(data,start,stop)

%% Get different variables

if(~exist('start', 'var'))
    start = 1;
else
    start = start*1000;
    startcheck = data{data{:,6}>start,6};
    start = find(data{:,6}==min(startcheck));
end

if(~exist('stop', 'var'))
    stop = size(data,1);
else
    stop = stop*1000;
    stopcheck = data{data{:,6}<stop,6};
    stop = find(data{:,6}==max(stopcheck));
end

rpm     = data{start:stop,1};
voltage = data{start:stop,2};
current = data{start:stop,3};
input   = data{start:stop,4};
pwm     = data{start:stop,5};
time    = data{start:stop,6};
type    = data{start:stop,7};

voltage(voltage<1000)=NaN;
current(current<1000)=NaN;

addnan=zeros(length(time),1);

for i=2:length(time)
    if time(i)>time(i-1)+500
        addnan(i)=1;
    end
end

time_new = zeros(length(time)+sum(addnan),1);
rpm_new = time_new;
voltage_new = time_new;
current_new = time_new;
input_new = time_new;
pwm_new = time_new;

corr=0;
for i=1:length(time)
    if addnan(i)==1
        time_new(i+corr)=NaN;
        rpm_new(i+corr)=NaN;
        voltage_new(i+corr)=NaN;
        current_new(i+corr)=NaN;
        input_new(i+corr)=NaN;
        pwm_new(i+corr)=NaN;
        corr=corr+1;
        time_new(i+corr)=time(i);
        rpm_new(i+corr)=rpm(i);
        voltage_new(i+corr)=voltage(i);
        current_new(i+corr)=current(i);
        input_new(i+corr)=input(i);
        pwm_new(i+corr)=pwm(i);
    else
        time_new(i+corr)=time(i);
        rpm_new(i+corr)=rpm(i);
        voltage_new(i+corr)=voltage(i);
        current_new(i+corr)=current(i);
        input_new(i+corr)=input(i);
        pwm_new(i+corr)=pwm(i);
    end
end

clear time rpm voltage current input pwm
time = time_new;
rpm = rpm_new;
voltage = voltage_new;
current = current_new;
input = input_new;
pwm = pwm_new;


%% Conversion of data (+accounting for controller type)
speed = rpm.*(60/4.857143)*(11/100000)*3.141592654;
voltage = voltage.*0.0001875*(100+33)/33.1;
current = (current.*0.0001875*(22+33)/33-2.505)/.066;

if (type(1)==1) %speed controller: input provides desired car speed
    input1 = ceil(input.*120/3.27);
    input1 = input1.*(60/4.857143)*(11/100000)*3.141592654;
    input1(input<=25) = NaN;
elseif (type(1)==2) %current controller: input provides desired current draw
    input1 = input.*20/255+0.4;
    input1(input<0) = NaN;
end

time = time./1000; 

%% Check for reverse segments in the data

if(type(1)==2)
    negative = zeros(length(input),1);
    input(isnan(input))=0;
    for i=2:length(input)-1
        if(input(i)<0&&input(i-1)>=0)
            negative(i)=1;
        end
        if(input(i)<0&&input(i+1)>=0)
            if(negative(i)~=1)
                negative(i)=2;
                if(isempty(find(negative(1:i)==1,1)))
                    negative(1)=1;
                end
            else
                negative(i)=0;
            end   
        end
    end
    if(sum(negative)>0)
        for i=1:length(input)
            pos=length(input)-(i-1);
            if (negative(pos)>0)
                if (negative(pos)==2)
                    break
                elseif (negative(pos)==1)
                    negative(end)=2;
                    break
                end
            end
        end
    end
end

if(type(1)==1)
    negative = zeros(length(input),1);
    input(isnan(input))=0;
    for i=2:length(input)-1
        if(input(i)<=25&&input(i-1)>25)
            negative(i)=1;
        end
        if(input(i)<=25&&input(i+1)>25)
            if(negative(i)~=1)
                negative(i)=2;
                if(isempty(find(negative(1:i)==1,1)))
                    negative(1)=1;
                end
            else
                negative(i)=0;
            end   
        end
    end
    if(sum(negative)>0)
        for i=1:length(input)
            pos=length(input)-(i-1);
            if (negative(pos)>0)
                if (negative(pos)==2)
                    break
                elseif (negative(pos)==1)
                    negative(end)=2;
                    break
                end
            end
        end
    end
end


%% Plot data

if(type(1)==0)
    figure('name','Data','position',[25 25 1600 1000]);    
    subplot(2,2,1)
    plot(time,voltage,'color',[0.2 0.2 0.8],'linewidth',2);
    ylabel('Voltage / V','fontsize',14,'fontweight','bold')
    ylim([min(voltage)-0.1 max(voltage)+0.1])
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;      
    title('Voltage')
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')  
    subplot(2,2,2)
    plot(time,current,'color',[0.2 0.2 0.8],'linewidth',2);
    ylabel('Current / A','fontsize',14,'fontweight','bold')
    ylim([min(current)-0.5 max(current)+0.5])
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;      
    title('Current')
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')   
    subplot(2,2,3)
    plot(time,speed,'color',[0.2 0.2 0.8],'linewidth',2) 
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;  
    title('Speed')
    ylabel('Speed / km/h','fontsize',14,'fontweight','bold')
    ylim([min(speed)-0.5 max(speed)+1])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')
    subplot(2,2,4)
    plot(time,pwm,'color',[0.2 0.2 0.8],'linewidth',2)
    hold on
    plot(time,abs(input),'color',[0.8 0.2 0.2],'LineWidth',2)
    hold off
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];    
    title('PWM duty cycle')
    ylabel('PWM duty cycle / 8 bit','fontsize',14,'fontweight','bold')
    ylim([min(abs(input))-5 max(input)+5])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')
    legend('Actual','Desired','location','northwest')
    sgtitle("Open loop motor control (PWM)",'fontsize',14,'fontweight','bold')
elseif(type(1)==1)
    figure('name','Data','position',[25 25 1600 1000]);    
    subplot(2,2,1)
    plot(time,voltage,'color',[0.2 0.2 0.8],'linewidth',2);
    ylabel('Voltage / V','fontsize',14,'fontweight','bold')
    ylim([min(voltage)-0.1 max(voltage)+0.1])
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;      
    title('Voltage')
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')  
    subplot(2,2,2)
    plot(time,current,'color',[0.2 0.2 0.8],'linewidth',2);
    ylabel('Current / A','fontsize',14,'fontweight','bold')
    ylim([min(current)-0.5 max(current)+0.5])
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;      
    title('Current')
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')  
    subplot(2,2,3)
    plot(time,pwm,'color',[0.2 0.2 0.8],'linewidth',2) 
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;    
    title('PWM duty cycle')
    ylabel('PWM duty cycle / 8 bit','fontsize',14,'fontweight','bold')
    ylim([min(pwm)-5 max(pwm)+5])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')
    subplot(2,2,4)
    plot(time,speed,'color',[0.2 0.2 0.8],'linewidth',2)
    hold on
    plot(time,abs(input1),'color',[0.8 0.2 0.2],'LineWidth',2)

    if(sum(negative)>0)
        if(min(abs(input1))<=min(speed))
            yindex1=min(abs(input1));
        else
            yindex1=min(speed);
        end
         if(max(abs(input1))>=max(speed))
            yindex2=max(abs(input1));
        else
            yindex2=max(speed);
        end
        y_fill=[yindex1-1,yindex1-1,yindex2+2,yindex2+2];
        %y_fill=[-0.25,-0.25,0.5,0.5];
        for i=1:length(input)
            if(negative(i)==1)
                xindex1=i;
            elseif(negative(i)==2)
                xindex2=i;
                x_fill=[time(xindex1),time(xindex2),time(xindex2),time(xindex1)];
                fill(x_fill,y_fill,[0.8 0.8 0.2],'EdgeColor','none','FaceAlpha',0.5);
            end
        end
    end

    hold off
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;
    title('Speed')
    ylabel('Speed / km/h','fontsize',14,'fontweight','bold')
    if (min(speed)<min(input1))
        if (max(speed)>max(input1))
            ylim([min(abs(speed))-1 max(abs(speed))+2])
        else
            ylim([min(abs(speed))-1 max(abs(input1))+2])
        end
    else
        if (max(speed)>max(input1))
            ylim([min(abs(input1))-1 max(abs(speed))+2])
        else
            ylim([min(abs(input1))-1 max(abs(input1))+2])
        end        
    end
    %ylim([min(abs(input1))-0.5 max(abs(input1))+10])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')
    if(sum(negative>0))
        legend('Actual','Desired','Off','location','northwest')
    else
        legend('Actual','Desired','location','northwest')        
    end
    sgtitle("Closed loop motor control (speed)",'fontsize',14,'fontweight','bold')
else
    figure('name','Data','position',[25 25 1600 1000]);  
    subplot(2,2,1)
    plot(time,voltage,'color',[0.2 0.2 0.8],'linewidth',2);
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;  
    title('Voltage')
    ylabel('Voltage / V','fontsize',14,'fontweight','bold')
    ylim([min(voltage)-0.1 max(voltage)+0.1])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')

    subplot(2,2,2)
    plot(time,pwm,'color',[0.2 0.2 0.8],'linewidth',2)
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;  
    title('PWM duty cycle')
    ylabel('PWM duty cycle / 8 bit','fontsize',14,'fontweight','bold')
    ylim([min(pwm)-5 max(pwm)+5])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')

    subplot(2,2,3)
    plot(time,speed,'color',[0.2 0.2 0.8],'linewidth',2) 
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;  
    title('Speed')
    ylabel('Speed / km/h','fontsize',14,'fontweight','bold')
    ylim([min(speed)-0.5 max(speed)+1])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')

    subplot(2,2,4)
    plot(time,current,'color',[0.2 0.2 0.8],'linewidth',2)
    hold on
    plot(time,abs(input1),'color',[0.8 0.2 0.2],'LineWidth',2)

    if(min(abs(input1))<=min(current))
        yindex1=min(abs(input1));
    else
        yindex1=min(current);
    end
     if(max(abs(input1))>=max(current))
        yindex2=max(abs(input1));
    else
        yindex2=max(current);
    end
    if(sum(negative)>0)
        y_fill=[yindex1-0.5,yindex1-0.5,yindex2+0.5,yindex2+0.5];
        %y_fill=[-1,-1,5,5];
        for i=1:length(input)
            if(negative(i)==1)
                xindex1=i;
            elseif(negative(i)==2)
                xindex2=i;
                x_fill=[time(xindex1),time(xindex2),time(xindex2),time(xindex1)];
                fill(x_fill,y_fill,[0.8 0.8 0.2],'EdgeColor','none','FaceAlpha',0.5);
            end
        end
    end

    hold off
    grid on
    ax = gca;
    ax.FontSize = 12;
    ax.LineWidth = 1.5;
    ax.GridColor = [0 0 0];
    ax.GridLineWidth = 1;
    ylabel('Current / A','fontsize',14,'fontweight','bold')
    ylim([yindex1-0.5 yindex2+0.5])
    xlabel('Runtime / s','fontsize',14,'fontweight','bold')   
    title('Current')
    if(sum(negative>0))
        legend('Actual','Desired','Off','location','northwest')
    else
        legend('Actual','Desired','location','northwest')        
    end
    sgtitle("Closed loop motor control (torque)",'fontsize',14,'fontweight','bold')
end

% figure('name','Power','position',[25 25 700 500]);
% plot(time,voltage.*current,'color',[0.2 0.2 0.8],'linewidth',2)
% title('Power')
% grid on
% ax = gca;
% ax.FontSize = 12;
% ax.LineWidth = 1.5;
% ax.GridColor = [0 0 0];
% ax.GridLineWidth = 1;
% xlabel('Runtime / s','fontsize',14,'fontweight','bold') 
% ylabel('Power / W','fontsize',14,'fontweight','bold') 


end