function affichage( l1, l2, th1min, th1max, th2min, th2max, xo, yo, ro)

varstr(1,:) = '  l1  ';
varstr(2,:) = '  l2  ';
varstr(3,:) = 'th1min';
varstr(4,:) = 'th1max';
varstr(5,:) = 'th2min';
varstr(6,:) = 'th2max';
varstr(7,:) = '  xo  ';
varstr(8,:) = '  yo  ';
varstr(9,:) = '  ro  ';

varstr(10,:) = '  phi ';

vi = zeros(1,6);
vi(1) = l1;
vi(2) = l2;
vi(3) = th1min;
vi(4) = th1max;
vi(5) = th2min;
vi(6) = th2max;
vi(7) = xo;
vi(8) = yo;
vi(9) = ro;

vi(10) = 0;
xpw=0;ypw=0;phi=0;place=0;

scsz = get(0,'ScreenSize');

%Open figure
fig = figure('FileName','Enveloppe_robot_RR',...
        'Name','Enveloppe de l''espace de travail du robot RR',...
        'NumberTitle','off',...
        'OuterPosition',[0 40 scsz(3)+1 scsz(4)-39]);
    
    %Graphics for cartesian workspace
    graph = axes('OuterPosition',[0.3 0 0.75 1],'tag','tggraph');
    
    % Graphics for joint space
    graphjoint = axes('OuterPosition',[0.15 0.6 0.22 0.4],...
        'XLim',[-185 185],'YLim',[-185 185],'tag','tggraphjoint');
    
    calculatedraw(l1, l2, th1min, th1max, th2min, th2max, xo, yo, ro, fig, graph, graphjoint);
    
    % built slider and text to change parameters
    for i = 1:6
        slider(i) = uicontrol('style','slider',...
            'Units','normalized',...
            'Position',[0.05*(i-1)+0.02 0.02 0.03 0.3],...
            'Callback',@slide,...
            'tag',['tslider',num2str(i)],...
            'Min',0,'Max',10,...
            'Value',vi(i)); %#ok<*AGROW>

        edtext(i) = uicontrol('style','edit',...
            'Units','normalized',...
            'Position',[0.05*(i-1)+0.0125 0.33 0.045 0.04],...
            'String',num2str(vi(i)),...
            'tag',['tedtext',num2str(i)],...
            'Callback',@editvar);
        
        texts(i) = uicontrol('style','text',...
            'Units','normalized',...
            'Position',[0.05*(i-1)+0.0125 0.37 0.045 0.03],...
            'String',varstr(i,:),...
            'tag',['tedtext',num2str(i)],...
            'BackgroundColor',get(fig,'Color'));
        
        if i>0 && i<3
            set(slider(i),'Min',0);
            set(slider(i),'Max',20);
        else
            set(slider(i),'Min',-180);
            set(slider(i),'Max',180);            
        end
    end
    
    for i = 7:9
       edtext(i) = uicontrol('style','edit',...
            'Units','normalized',...
            'Position',[0.05*(i-7)+0.0125 0.43 0.045 0.04],...
            'String',num2str(vi(i)),...
            'tag',['tedtext',num2str(i)],...
            'Callback',@editvar);
        
        texts(i) = uicontrol('style','text', ...
            'Units','normalized',...
            'Position',[0.05*(i-7)+0.0125 0.47 0.045 0.03],...  
            'String',varstr(i,:),...
            'tag',['tedtext',num2str(i)],...
            'BackgroundColor',get(fig,'Color'));       
    end

         edtext(10) = uicontrol('style','edit',...
            'Units','normalized',...
            'Position',[0.05*(1.5)+0.0125 0.76 0.045 0.04],...
            'String',num2str(vi(10)),...
            'tag',['tedtext',num2str(10)],...
            'Callback',@editvar);
        
         texts(10) = uicontrol('style','text', ...
            'Units','normalized',...
            'Position',[0.05*(1.5)+0.0125 0.8 0.045 0.03],...  
            'String',varstr(10,:),...
            'tag',['tedtext',num2str(10)],...
            'BackgroundColor',get(fig,'Color'));         
    
    buttonplaceo = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.52 0.3 0.04],...
        'String','Place obstacle with mouse',...
        'tag','bplaceo',...
        'Callback',@placeo,...
        'BackgroundColor',get(fig,'Color'));%#ok<*NASGU>
    
    buttonplacerobot = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.58 0.22 0.04],...
        'String','Place robot end effector with mouse',...
        'tag','bplaceRobot',...
        'Callback',@placeRobot,...
        'BackgroundColor',get(fig,'Color'));
    
    buttonplacePath = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.64 0.15 0.04],...        
        'String','Path in workspace',...
        'tag','bplaceP',...
        'Callback',@placeP,...
        'BackgroundColor',get(fig,'Color'));
    
    buttonplaceJPath = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.7 0.15 0.04],...
        'String',['Path in joint space'],...
        'tag','bplaceJP',...
        'Callback',@placeJP,...
        'BackgroundColor',get(fig,'Color'));
    
    buttonplacePart = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.76 0.08 0.04],...
        'String',['Place part in Workspace'],...
        'tag','bplacePW',...
        'Callback',@placePW,...
        'BackgroundColor',get(fig,'Color'));
    
    buttonplaceHidePart = uicontrol('style','radiobutton',...
        'Units','normalized',...
        'Position',[0.0125 0.82 0.08 0.04],...
        'String',['Hide part'],...
        'tag','bplacePW',...
        'Callback',@Hidepart,...
        'BackgroundColor',get(fig,'Color'));   
    
    textcom = uicontrol('style','text',...
        'Units','normalized',...
        'Position',[0.4 0.02 0.5 0.06],...
        'String','',...
        'tag','tgtextcom','Callback',@placeRobot,...
        'BackgroundColor',get(fig,'Color'),...
        'Visible','off','FontUnits','normalized',...
        'ForegroundColor','r', 'FontSize',0.3);
        
    
    %Create a function which links amin and cursors and display the new
    %graph

    function slide(hObject, eventdata)
        set(textcom,'Visible','off');
        
        k = str2double(strrep(get(hObject,'tag'),'tslider',''));

        %On s'assure que le min reste bien inférieur au max avent de
        %changer la valeur d'un angle limite.
        if k > 2
            if mod(k,2)==1 
                if get(slider(k),'Value')>str2num(get(edtext(k+1),'String'))
                    vc = str2num(get(edtext(k),'String'));
                    set(slider(k),'Value',vc);
                else
                    vc = get(slider(k),'Value');
                end
            else
                if get(slider(k),'Value')<str2num(get(edtext(k-1),'String'))
                    vc = str2num(get(edtext(k),'String'));
                    set(slider(k),'Value',vc);
                else
                    vc = get(slider(k),'Value');
                end
            end
        else
            vc = get(slider(k),'Value');
        end

        set(edtext(k),'String',vc);
        %evaluate values of parameters written
        for j = 1:9
            vi(j) = str2num(get(edtext(j),'String'));
        end

        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;
    end
    
    %Function which enable to edit values of variables in the edit text area.
    function editvar(hObject, eventdata) %#ok<*INUSD>
        set(textcom,'Visible','off');
        
        k = str2double(strrep(get(hObject,'tag'),'tedtext',''));

        %On s'assure que le min reste bien inférieur au max avent de
        %changer la valeur d'un angle limite.
        if k > 2 && k < 7
            if mod(k,2)==1 
                if str2num(get(edtext(k),'String'))>=str2num(get(edtext(k+1),'String'))
                    vc = get(slider(k),'Value');
                    set(edtext(k),'String',num2str(vc));
                    set(textcom,'Visible','on','String',strcat('Theta',num2str((k-2)/2),...
                        'min must be lower than ','theta',num2str((k-2)/2),'max'));
                else
                    vc = str2num(get(edtext(k),'String'));
                end
            else
                if str2num(get(edtext(k),'String'))<=str2num(get(edtext(k-1),'String'))
                    vc = get(slider(k),'Value');
                    set(edtext(k),'String',num2str(vc));
                    set(textcom,'Visible','on','String',strcat('Theta',num2str((k-2)/2),...
                        'max must be higher than ','theta',num2str((k-2)/2),'min'));
                else
                    vc = str2num(get(edtext(k),'String'));
                end
            end
        else
            vc = str2num(get(edtext(k),'String')); %#ok<*ST2NM>
        end
        
        if k < 7
            set(slider(k),'Value',vc);
        end
        
        %evaluate values of parameters written
        for j = 1:10
            vi(j) = str2num(get(edtext(j),'String'));
        end
        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;
    end

    %Function which enable to place the part with mouse.
    function placePW(hObject, eventdata)
        place=1;
        vi(10) = str2num(get(edtext(10),'String')); 
        phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
         if (get(hObject,'Value') == get(hObject,'Max'))
             
            but = 1;     
            while but == 1
                % Radio button is selected, take appropriate action
                xpws=xpw;ypws=ypw;
                part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
                set(fig,'CurrentAxes',graph);hold on;
                plot(graph, part);
                set(textcom,'Visible','on','String','Click on right button on the mouse to exit');
                [xpw ypw but] = ginput(1);
                v = zeros(1,9);
                %evaluate values of parameters written
                for j = 1:10
                    vi(j) = str2num(get(edtext(j),'String'));
                end
                calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
                vi(10) = str2num(get(edtext(10),'String')); 
            end
            
         end
         xpw=xpws;ypw=ypws;
         set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
         set(hObject,'Value',0);set(textcom,'Visible','off');
    end

   %Function which enable to hide the part with mouse.
    function Hidepart(hObject, eventdata)
                if (get(hObject,'Value') == get(hObject,'Max'))            
                %evaluate values of parameters written
                place=0; 
                for j = 1:9
                    vi(j) = str2num(get(edtext(j),'String'));
                end
                calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
         end
         set(hObject,'Value',0);set(textcom,'Visible','off');
    end   
    %Function which enable to place the obstacle with mouse.
    function placeo(hObject, eventdata)

         if (get(hObject,'Value') == get(hObject,'Max'))
             
            but = 1;     
            while but == 1
                % Radio button is selected, take appropriate action
                set(edtext(7),'String',num2str(xo));
                set(edtext(8),'String',num2str(yo));
                v = zeros(1,9);
                %evaluate values of parameters written
                for j = 1:9
                    vi(j) = str2num(get(edtext(j),'String'));
                end
                calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
                set(textcom,'Visible','on','String','Click on right button on the mouse to exit');
                [xo yo but] = ginput(1);
            end
            
         end
         set(hObject,'Value',0);set(textcom,'Visible','off');
    end
    
    %Function which enable to place the position of the effector of the robot with mouse.
    function placeRobot(hObject, eventdata)
        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;       
        if (get(hObject,'Value') == get(hObject,'Max'))
            set(textcom,'Visible','on','String','Click on right button on your mouse to exit');
            [x y but] = ginput(1);
            [elbSta, robotArmsc, jrobotArmsc] = drawrobot( x, y, vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom);
            robotArmso = robotArmsc;
            jrobotArmso = jrobotArmsc;                  
            while but == 1                       
                [elbSta, robotArmsc, jrobotArmsc] = drawrobot( x, y, vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom, robotArmso, jrobotArmso);
                %set(textcom,'Visible','on','String','Click on right button on your mouse to exit');
                robotArmso = robotArmsc;
                jrobotArmso = jrobotArmsc;
            [x y but] = ginput(1);    
            end
        end
        if ishandle(robotArmso(1))
            delete(robotArmso(1));
        end
        if ishandle(robotArmso(2))
            delete(robotArmso(2));
        end
        set(hObject,'Value',0);
        set(hObject,'Value',0);set(textcom,'Visible','off');
        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;
    end

    %Place path on cartesian space and draw motion
    function placeP(hObject, eventdata)    
        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;       
        if (get(hObject,'Value') == get(hObject,'Max'))
            set(textcom,'Visible','on','String','Click on right button for last point');
            %set(textcom,'Visible','off');
            %evaluate values of parameters written
            for j = 1:9
                vi(j) = str2num(get(edtext(j),'String'));
            end
            
             %calculate the path
            step = 0:0.05:1;
            k=1;
            [x(k) y(k) but] = ginput(1);
            while but == 1  
               k=k+1;
               [x(k) y(k) but] = ginput(1);
               xcp(k-1,:) = (1-step)*x(k-1)+step*x(k); ycp(k-1,:) = (1-step)*y(k-1)+step*y(k);
            end;
            %there are two possibilites : robot has been drawn and it must be erased or not
            xc=xcp(1,:);yc=ycp(1,:);
            for t=2:k-1
                xc = [xc xcp(t,:)];
                yc = [yc ycp(t,:)];
            end %for 1:k-1   
                isstart = 1; col = [0 0];
                for i = 1:(k-1)*length(step)
                    if isstart == 1
                        [elbSta, robotArmsc, jrobotArmsc] = drawrobot( xc(i), yc(i), vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom);
                        robotArmso = robotArmsc;
                        jrobotArmso = jrobotArmsc;
                        isstart = 0;
                    else
                        [elbSta, robotArmsc, jrobotArmsc] = drawrobot( xc(i), yc(i), vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom, robotArmso, jrobotArmso);
                        robotArmso = robotArmsc;
                        jrobotArmso = jrobotArmsc;
                    end
                    
                    % Test if the path is possible
                    if i > 1
                        elbStac = elbSta.*elbStac;
                    else
                        elbStac = elbSta;
                    end
                    
                    % check collision
                    mc = get(robotArmso,'LineStyle');
                    for j = 1:2
                        if strcmp(mc(j),':') == 1
                            col(j) = 1;
                        end
                    end
                end
                
                %erase the old picture of robot
                delete(robotArmso(1)); delete(robotArmso(2)); delete(jrobotArmso);
                
                %if the path is possible, load the graph with arms in M.
                if (elbStac(1) == 0 && elbStac(2) == 0)
                    set(fig,'CurrentAxes',graph);
                    hold on
                    set(textcom,'Visible','on','String','This path is not possible');
                else
                    if (col(1)+(1-elbStac(1)) == 0 || col(2)+(1-elbStac(2)) == 0)
                        isstart = 1;
                        for i = 1:(k-1)*length(step)
                            if isstart == 1
                                [elbSta, robotArmsc, jrobotArmsc] = drawrobot( xc(i), yc(i), vi(1), vi(2), vi(3), vi(4), elbStac(2)*vi(5), elbStac(1)*vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom);
                                M(i) = getframe;
                                robotArmso = robotArmsc;
                                jrobotArmso = jrobotArmsc;
                                isstart = 0;
                            else
                                [elbSta, robotArmsc, jrobotArmsc] = drawrobot(xc(i), yc(i),  vi(1), vi(2), vi(3), vi(4), elbStac(2)*vi(5), elbStac(1)*vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint, textcom, robotArmso, jrobotArmso);
                                M(i) = getframe;
                                robotArmso = robotArmsc;
                                jrobotArmso = jrobotArmsc;
                            end
                        end
                        %play the movie with all graph
                        movie(M);
                        
                        delete(robotArmso(1)); delete(robotArmso(2)); delete(jrobotArmso);
                    else
                        set(textcom,'Visible','on','String','The obstacle is on the path');
                    end
                end
            end
            set(hObject,'Value',0);  
    %calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;
    end

    %Place path on joint space and draw motion
    function placeJP(hObject, eventdata)
        calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
        if place==1
            phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
            part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
            set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
        end;       
        if (get(hObject,'Value') == get(hObject,'Max'))
            set(textcom,'Visible','off');
            %evaluate values of parameters written
            for j = 1:9
                vi(j) = str2num(get(edtext(j),'String'));
            end
            
            step = 0:0.05:1;
            k=1;outbounds=0;
            [th1(k) th2(k) but] = ginput(1);
            while but == 1  
               k=k+1;
               [th1(k) th2(k) but] = ginput(1);
               if (th1(k) > vi(4) || th1(k) < vi(3) || th1(k-1) > vi(4) || th1(k-1) < vi(3))
                   set(textcom,'Visible','on','String','Theta 1 is out of bounds');
                   but=0;outbounds=1;
               elseif (th2(k) > vi(6) || th2(k) < vi(5) || th2(k-1) > vi(6) || th2(k-1) < vi(5))
                   set(textcom,'Visible','on','String','Theta 2 is out of bounds');
                   but=0;outbounds=1;
               else
                   pathj = plot(graphjoint,th1,th2,'ok-');
                   th1cp(k-1,:) = (1-step)*th1(k-1)+step*th1(k); th2cp(k-1,:) = (1-step)*th2(k-1)+step*th2(k);
               end;
            end;
            if outbounds==0
                th1c=th1cp(1,:);th2c=th2cp(1,:);
                for t=2:k-1
                    th1c = [th1c th1cp(t,:)];
                    th2c = [th2c th2cp(t,:)];
                end %for 1:k-1
                co = pi/180;
                %check if the obstacle is on the path
                for i = 1:(k-1)*length(step)
                    if testCol(co*th1c(i),co*th2c(i),vi(1),vi(2),vi(7),vi(8),vi(9)) ~= 0
                        set(textcom,'Visible','on','String','The obstacle is on the path');
                        step = [];
                    end
                end
                %there are two possibilites : robot has been drawn and it
                %must be erased or not
                isstart = 1;
                for i = 1:(k-1)*length(step)
                    if isstart == 1
                        set(fig,'CurrentAxes',graph);
                        hold on
                        robotc = plot(graph,[0 vi(1)*cos(co*th1c(i)) vi(1)*cos(co*th1c(i))+vi(2)*cos(co*th1c(i)+co*th2c(i))],...
                            [0 vi(1)*sin(co*th1c(i)) vi(1)*sin(co*th1c(i))+vi(2)*sin(co*th1c(i)+co*th2c(i))],'k-');
                        JM(i) = getframe;
                        isstart = 0;
                    else
                        set(fig,'CurrentAxes',graph);
                        hold on
                        delete(robotc);
                        robotc = plot(graph,[0 vi(1)*cos(co*th1c(i)) vi(1)*cos(co*th1c(i))+vi(2)*cos(co*th1c(i)+co*th2c(i))],...
                            [0 vi(1)*sin(co*th1c(i)) vi(1)*sin(co*th1c(i))+vi(2)*sin(co*th1c(i)+co*th2c(i))],'k-');
                        JM(i) = getframe;
                    end
                end
                if ~isempty(step)
                    %play the movie with all graph
                    movie(JM);
                    delete(robotc);
                    calculatedraw(vi(1), vi(2), vi(3), vi(4), vi(5), vi(6), vi(7), vi(8), vi(9), fig, graph, graphjoint);
                end
                delete(pathj);
            end
            set(hObject,'Value',0);
            if place==1
                phir=(vi(10)*pi)/180;cosp=cos(phir);sinp=sin(phir);
                part = polyshape([xpw xpw+1*cosp xpw+1*cosp-0.5*sinp xpw+0.5*cosp-0.5*sinp xpw+0.5*cosp-2*sinp xpw-2*sinp],[ypw ypw+1*sinp ypw+1*sinp+0.5*cosp ypw+0.5*sinp+0.5*cosp ypw+0.5*sinp+2*cosp ypw+2*cosp]);
                set(fig,'CurrentAxes',graph);hold on;plot(graph, part);
            end;
        end;
    end
end

