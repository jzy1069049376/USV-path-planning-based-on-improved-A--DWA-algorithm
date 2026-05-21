
clear; clc; close all;


img = imread('D:\jzy1068049376\coast1111.jpg');
if size(img,3) > 1
    img_gray = rgb2gray(img);
    img_bw = imbinarize(img_gray);
else
    img_bw = imbinarize(img);
end
img_bw = imresize(img_bw, 2);


scale = 10;
[h, w] = size(img_bw);
new_h = floor(h/scale);
new_w = floor(w/scale);
grid = false(new_h, new_w);

for i = 1:new_h
    for j = 1:new_w
        block = img_bw((i-1)*scale+1:i*scale, (j-1)*scale+1:j*scale);
        grid(i,j) = mean(block(:)) < 0.5;
    end
end
[map_h, map_w] = size(grid);
fprintf('栅格地图：%d行×%d列\n',map_h,map_w);


risk_map = zeros(map_h, map_w);
obs_radius = 8; 

for r = 1:map_h
    for c = 1:map_w
        if ~grid(r,c)
            for dr = -obs_radius:obs_radius
                for dc = -obs_radius:obs_radius
                    rr = r+dr; cc = c+dc;
                    if rr>=1&&rr<=map_h&&cc>=1&&cc<=map_w&&grid(rr,cc)
                        d = sqrt(dr^2+dc^2);
                        risk_map(rr,cc) = max(risk_map(rr,cc),exp(-d^2/(2*(obs_radius/2)^2)));
                    end
                end
            end
        end
    end
end

cost_imp = 1 + 15*risk_map;   
cost_std = ones(map_h, map_w); 


flow_base = [0, 0.8];
flow_density = 8;
flow_obs_range = 3;


inf_imp = 1;
inf_astar = 1;
inf_trad = 0;


figure('Name','选点','Position',[100,100,800,600]);
imshow(grid);
title('1 选起点');
axis on; hold on; grid on;


[c1, r1] = ginput(1);
[sr, sc] = calibrate(round(r1),round(c1),grid);
start = [sr, sc];
plot(c1,r1,'go','MarkerSize',12,'MarkerFaceColor','g');
text(c1+3,r1,'起点','FontSize',10);


title('2 选终点');
[c2, r2] = ginput(1);
[gr, gc] = calibrate(round(r2),round(c2),grid);
goal = [gr, gc];
plot(c2,r2,'r^','MarkerSize',12,'MarkerFaceColor','r');
text(c2+3,r2,'终点','FontSize',10);


obs_num = 1;
obs_s = zeros(obs_num,2);
obs_g = zeros(obs_num,2);
obs_color = {'m'};

for k = 1:obs_num
    title(sprintf('3.%d 选障碍%d起点',k,k));
    [ck1,rk1] = ginput(1);
    [or1,oc1] = calibrate(round(rk1),round(ck1),grid);
    obs_s(k,:) = [or1,oc1];
    plot(ck1,rk1,'s','Color',obs_color{k},'MarkerSize',12,'MarkerFaceColor',obs_color{k});
    
    title(sprintf('3.%d 选障碍%d终点',k,k));
    [ck2,rk2] = ginput(1);
    [or2,oc2] = calibrate(round(rk2),round(ck2),grid);
    obs_g(k,:) = [or2,oc2];
    plot(ck2,rk2,'s','Color',obs_color{k},'MarkerSize',12,'MarkerFaceColor',obs_color{k});
end
input('选完回车开始仿真...\n');


start(start<1)=1;
start(1)=min(start(1),map_h);
start(2)=min(start(2),map_w);

goal(goal<1)=1;
goal(1)=min(goal(1),map_h);
goal(2)=min(goal(2),map_w);

for k=1:obs_num
    obs_s(k,obs_s(k,:)<1)=1;
    obs_s(k,1)=min(obs_s(k,1),map_h);
    obs_s(k,2)=min(obs_s(k,2),map_w);
    obs_g(k,obs_g(k,:)<1)=1;
    obs_g(k,1)=min(obs_g(k,1),map_h);
    obs_g(k,2)=min(obs_g(k,2),map_w);
end


obs_path = cell(obs_num,1);
for k=1:obs_num
    [p_raw,~,~] = AStar8(grid,cost_std,obs_s(k,:),obs_g(k,:));
    p_clean = clean_path(p_raw,0.8);
    p_simple = douglas(p_clean,2.2);
    obs_path{k} = smooth_path(p_simple,4);
    if isempty(obs_path{k})
        error('障碍路径失败');
    end
end


path1 = [start;goal];


[p2_raw,~,~] = AStar8(grid,cost_std,start,goal);
p2_clean = clean_path(p2_raw,0.4);
p2_simple = douglas(p2_clean,1.0);
path2 = smooth_path(p2_simple,2);


[p3_raw,~,~] = AStar8(grid,cost_imp,start,goal);
p3_clean = clean_path(p3_raw,0.8);
p3_simple = douglas(p3_clean,2.2);
path3 = smooth_path(p3_simple,4);

if isempty(path2),error('A*路径失败');end
if isempty(path3),error('改进A*路径失败');end



boat1.pos = start; boat1.history = start;
boat1.last_dir = [0,0]; boat1.done = 0;
boat1.len = 0; boat1.step = NaN; boat1.min_d = inf;


boat2.pos = start; boat2.history = start;
boat2.last_dir = [0,0]; boat2.done = 0;
boat2.path = path2; boat2.len = 0; boat2.step = NaN; boat2.min_d = inf;


boat3.pos = start; boat3.history = start;
boat3.last_dir = [0,0]; boat3.done = 0;
boat3.path = path3; boat3.len = 0; boat3.step = NaN; boat3.min_d = inf;


speed = 0.15;
max_step = 1200;
safe1=1.0; safe2=1.3; safe3=1.6;

obs_pos = zeros(obs_num,2);
obs_cnt = zeros(obs_num,1);
obs_idx = ones(obs_num,1);
obs_update = [2];

for k=1:obs_num
    obs_pos(k,:) = obs_path{k}(obs_idx(k),:);
end


figure('Name','三算法对比','Position',[150,100,900,700]);

for step = 1:max_step
    if boat1.done && boat2.done && boat3.done
        break;
    end

    
    if ~boat1.done
        tar1 = goal;
        next1 = DWA1(boat1.pos,tar1,obs_pos,grid,safe1,boat1.last_dir,inf_trad);
        boat1.len = boat1.len + norm(next1-boat1.pos);
        boat1.last_dir = next1-boat1.pos;
        boat1.pos = next1;
        boat1.history = [boat1.history;next1];
        for k=1:obs_num
            boat1.min_d = min(boat1.min_d,norm(boat1.pos-obs_pos(k,:)));
        end
        if norm(boat1.pos-goal) < 1
            boat1.done = 1; boat1.step = step;
        end
    end

    
    if ~boat2.done
        tar2 = get_target(boat2.pos,boat2.path,goal,2);
        next2 = DWA2(boat2.pos,tar2,obs_pos,grid,safe2,boat2.last_dir,inf_astar);
        boat2.len = boat2.len + norm(next2-boat2.pos);
        boat2.last_dir = next2-boat2.pos;
        boat2.pos = next2;
        boat2.history = [boat2.history;next2];
        for k=1:obs_num
            boat2.min_d = min(boat2.min_d,norm(boat2.pos-obs_pos(k,:)));
        end
        if norm(boat2.pos-goal) < 1
            boat2.done = 1; boat2.step = step;
        end
    end

    
    if ~boat3.done
        tar3 = get_target(boat3.pos,boat3.path,goal,4);
        next3 = DWA3(boat3.pos,tar3,obs_pos,grid,safe3,boat3.last_dir,inf_imp);
        boat3.len = boat3.len + norm(next3-boat3.pos);
        boat3.last_dir = next3-boat3.pos;
        boat3.pos = next3;
        boat3.history = [boat3.history;next3];
        for k=1:obs_num
            boat3.min_d = min(boat3.min_d,norm(boat3.pos-obs_pos(k,:)));
        end
        if norm(boat3.pos-goal) < 1
            boat3.done = 1; boat3.step = step;
        end
    end

    
    for k=1:obs_num
        obs_cnt(k)=obs_cnt(k)+1;
        if obs_cnt(k)>=obs_update(k)
            obs_idx(k)=obs_idx(k)+1;
            if obs_idx(k)>size(obs_path{k},1)
                obs_idx(k)=1;
            end
            obs_pos(k,:)=obs_path{k}(obs_idx(k),:);
            obs_cnt(k)=0;
        end
    end

    
    imshow(grid); hold on; grid on; axis equal;
    draw_flow(grid,flow_base,flow_density);
    for k=1:obs_num
        draw_obs_flow(obs_pos(k,:),flow_obs_range,flow_base,grid);
    end

    
    plot(boat1.history(:,2),boat1.history(:,1),'k-','LineWidth',1.8);
    plot(boat2.history(:,2),boat2.history(:,1),'-','Color',[0.85 0 0.85],'LineWidth',2);
    plot(path3(:,2),path3(:,1),'b--','LineWidth',1.5);
    plot(boat3.history(:,2),boat3.history(:,1),'b-','LineWidth',2.5);

    
    for k=1:obs_num
        plot(obs_path{k}(:,2),obs_path{k}(:,1),':','Color',obs_color{k},'LineWidth',1);
        plot(obs_pos(k,2),obs_pos(k,1),'s','Color',obs_color{k},'MarkerSize',10,'MarkerFaceColor',obs_color{k});
        text(obs_pos(k,2)+2,obs_pos(k,1),sprintf('障%d',k),'FontSize',8);
    end

    
    draw_boat(boat1.pos,goal,[0 0 0]);
    draw_boat(boat2.pos,goal,[0 0.75 0.75]);
    draw_boat(boat3.pos,goal,[0 0 1]);

    
    plot(start(2),start(1),'go','MarkerSize',12,'MarkerFaceColor','g');
    plot(goal(2),goal(1),'r^','MarkerSize',12,'MarkerFaceColor','r');

    
    title(sprintf('三算法对比 | step=%d',step));
    axis on; hold off;
    drawnow; pause(speed);
end


fprintf('\n===== 仿真结果 =====\n');
fprintf('传统DWA：步数=%d, 长度=%.2f, 最小障碍距=%.2f\n',boat1.step,boat1.len,boat1.min_d);
fprintf('A*-DWA：步数=%d, 长度=%.2f, 最小障碍距=%.2f\n',boat2.step,boat2.len,boat2.min_d);
fprintf('本文算法：步数=%d, 长度=%.2f, 最小障碍距=%.2f\n',boat3.step,boat3.len,boat3.min_d);

% 柱状图对比
figure('Position',[220,120,900,650]);
subplot(3,1,1);bar([boat1.step,boat2.step,boat3.step]);
xticklabels({'传统DWA','A*-DWA','本文算法'});ylabel('步数');grid on;
subplot(3,1,2);bar([boat1.len,boat2.len,boat3.len]);
xticklabels({'传统DWA','A*-DWA','本文算法'});ylabel('路径长度');grid on;
subplot(3,1,3);bar([boat1.min_d,boat2.min_d,boat3.min_d]);
xticklabels({'传统DWA','A*-DWA','本文算法'});ylabel('安全距离');grid on;
sgtitle('三算法性能对比');


function draw_boat(boat,tar,color)
    x=boat(2);y=boat(1);tx=tar(2);ty=tar(1);
    dx=tx-x;dy=ty-y;
    if norm([dx,dy])<1e-6, dx=1;dy=0;end
    ux=dx/norm([dx,dy]);uy=dy/norm([dx,dy]);
    L=2.5;W=0.7;
    head=[x+L*ux,y+L*uy];
    left=[x-L*ux/2+W*uy,y-L*uy/2-W*ux];
    rght=[x-L*ux/2-W*uy,y-L*uy/2+W*ux];
    patch([head(1),left(1),rght(1)],[head(2),left(2),rght(2)],color,'EdgeColor','k');
end

function res=douglas(path,eps)
    if size(path,1)<=2,res=path;return;end
    maxd=0;idx=2;
    for i=2:size(path,1)-1
        d=point2line(path(i,:),path(1,:),path(end,:));
        if d>maxd,maxd=d;idx=i;end
    end
    if maxd>eps
        left=douglas(path(1:idx,:),eps);
        right=douglas(path(idx:end,:),eps);
        res=[left(1:end-1,:);right];
    else
        res=[path(1,:);path(end,:)];
    end
end

function d=point2line(p,p1,p2)
    v1=p2-p1;v2=p-p1;
    d=norm(cross([v1,0],[v2,0]))/max(norm(v1),1e-6);
end

function res=clean_path(path,th)
    if nargin<2,th=0.8;end
    if isempty(path),res=[];return;end
    res=path(1,:);
    for i=2:size(path,1)
        if norm(path(i,:)-res(end,:))>th
            res=[res;path(i,:)];
        end
    end
end

function tar=get_target(boat,path,goal,step)
    if nargin<4,step=2;end
    if isempty(path),tar=goal;return;end
    [~,idx]=min(vecnorm(path-boat,2,2));
    t_idx=idx+step;
    if t_idx>=size(path,1),tar=goal;
    else tar=path(t_idx,:);end
end

function res=smooth_path(path,win)
    if nargin<2,win=2;end
    if size(path,1)<=win,res=path;return;end
    res=path;
    for i=win+1:size(path,1)-win
        res(i,:)=mean(path(i-win:i+win,:),1);
    end
end


function next=DWA1(pos,tar,obs,grid,safe,dir,inf)
    next=DWA_core(pos,tar,obs,grid,safe,dir,inf,1.7,12,0.85,0.10,0.05,0.00);
end
function next=DWA2(pos,tar,obs,grid,safe,dir,inf)
    next=DWA_core(pos,tar,obs,grid,safe,dir,inf,1.75,16,0.62,0.18,0.10,0.10);
end
function next=DWA3(pos,tar,obs,grid,safe,dir,inf)
    next=DWA_core(pos,tar,obs,grid,safe,dir,inf,1.8,16,0.50,0.20,0.10,0.20);
end


function next=DWA_core(cur,tar,obs,grid,safe,ldir,inf,step,ang_num,w1,w2,w3,w4)
    cur=reshape(cur,1,2);tar=reshape(tar,1,2);
    [h,w]=size(grid);
    ang=linspace(0,2*pi,ang_num+1)';ang(end)=[];
    dirs=[cos(ang),sin(ang)];
    cand=cur+dirs*step;
    valid=[];clear_s=[];

    for i=1:size(cand,1)
        c=cand(i,:);x=round(c(1));y=round(c(2));
        if x<1||x>h||y<1||y>w,continue;end
        if x-inf<1||x+inf>h||y-inf<1||y+inf>w,continue;end
        if any(grid(x-inf:x+inf,y-inf:y+inf)==0),continue;end
        
       
        col=0;
        for s=1:5
            inter=cur+(c-cur)*s/5;
            xi=max(1,min(round(inter(1)),h));
            yi=max(1,min(round(inter(2)),w));
            if ~grid(xi,yi),col=1;break;end
        end
        if col,continue;end

        min_d=inf;
        for k=1:size(obs,1),min_d=min(min_d,norm(c-obs(k,:)));end
        if min_d<safe,continue;end
        valid=[valid;c];clear_s=[clear_s;min_d];
    end

    if isempty(valid)
        next=cur+0.6*(tar-cur);
        next(1)=max(1,min(next(1),h));
        next(2)=max(1,min(next(2),w));
        return;
    end

    best=-inf;next=valid(1,:);
    clear_s=clear_s/max(max(clear_s),1e-6);
    for i=1:size(valid,1)
        c=valid(i,:);
        dist=1/max(norm(c-tar),1e-6);
        dir_c=c-cur;dir_t=tar-cur;
        ang=dot(dir_t,dir_c)/(norm(dir_t)*norm(dir_c)+1e-6);
        smooth=dot(dir_c,ldir)/(norm(dir_c)*norm(ldir)+1e-6);smooth=max(smooth,0);
        score=w1*dist+w2*ang+w3*smooth+w4*clear_s(i);
        if score>best,best=score;next=c;end
    end
end


function [path,iter,data] = AStar8(grid,cost,start,goal)
    path=[];iter=0;data=[];
    [h,w]=size(grid);
    sr=round(start(1));sc=round(start(2));
    gr=round(goal(1));gc=round(goal(2));

    if ~grid(sr,sc)||~grid(gr,gc),return;end
    open=[];close=false(h,w);
    pr=zeros(h,w);pc=zeros(h,w);
    g=inf(h,w);f=inf(h,w);
    g(sr,sc)=0;f(sr,sc)=hypot(sr-gr,sc-gc);
    open=[sr,sc,f(sr,sc)];
    dirs=[-1,0;1,0;0,-1;0,1;-1,-1;-1,1;1,-1;1,1];
    cost_d=[1,1,1,1,sqrt(2),sqrt(2),sqrt(2),sqrt(2)];
    find=0;best_f=inf;

    while ~isempty(open)
        iter=iter+1;
        [val,idx]=min(open(:,3));
        cr=open(idx,1);cc=open(idx,2);
        open(idx,:)=[];close(cr,cc)=1;
        if val<best_f,best_f=val;data=[data;iter,best_f];end
        if cr==gr&&cc==gc,find=1;break;end

        for d=1:size(dirs,1)
            nr=cr+dirs(d,1);nc=cc+dirs(d,2);
            if nr<1||nr>h||nc<1||nc>w||~grid(nr,nc)||close(nr,nc),continue;end
            tmp=g(cr,cc)+cost_d(d)*cost(nr,nc);
            if tmp<g(nr,nc)
                g(nr,nc)=tmp;
                f(nr,nc)=tmp+hypot(nr-gr,nc-gc);
                pr(nr,nc)=cr;pc(nr,nc)=cc;
                if ~in_open(open,nr,nc),open=[open;nr,nc,f(nr,nc)];end
            end
        end
    end

    if find
        cr=gr;cc=gc;
        while cr~=sr||cc~=sc
            path=[cr,cc;path];
            tr=pr(cr,cc);tc=pc(cr,cc);
            if tr==0&&tc==0,break;end
            cr=tr;cc=tc;
        end
        path=[sr,sc;path];
    end
end

function res=in_open(open,r,c)
    res=0;if isempty(open),return;end
    for i=1:size(open,1),if open(i,1)==r&&open(i,2)==c,res=1;return;end,end
end

function [r,c]=calibrate(r0,c0,grid)
    [h,w]=size(grid);
    if r0>=1&&r0<=h&&c0>=1&&c0<=w&&grid(r0,c0),r=r0;c=c0;return;end
    dirs=[-1,-1;-1,0;-1,1;0,-1;0,1;1,-1;1,0;1,1];
    for d=1:size(dirs,1)
        rn=r0+dirs(d,1);cn=c0+dirs(d,2);
        if rn>=1&&rn<=h&&cn>=1&&cn<=w&&grid(rn,cn),r=rn;c=cn;return;end
    end
    error('无可行区域');
end

function draw_flow(grid,base,density)
    [h,w]=size(grid);
    for i=density:density:h
        for j=density:density:w
            if grid(i,j)
                quiver(j,i,base(2),base(1),'Color',[0.6,0.8,1],'LineWidth',0.8);
            end
        end
    end
end

function draw_obs_flow(pos,range,base,grid)
    or=round(pos(1));oc=round(pos(2));[h,w]=size(grid);
    for dr=-range:range
        for dc=-range:range
            cr=or+dr;cc=oc+dc;
            if cr<1||cr>h||cc<1||cc>w||~grid(cr,cc),continue;end
            if abs(dr)>abs(dc)
                if dr<0,def=[base(1)*0.5,base(2)*1.2+0.3];
                else,def=[base(1)*0.5,base(2)*1.2-0.3];end
            else
                if dc<0,def=[base(1)*1.2+0.3,base(2)*0.5];
                else,def=[base(1)*1.2-0.3,base(2)*0.5];end
            end
            quiver(cc,cr,def(2),def(1),'Color',[0.2,0.4,0.8],'LineWidth',1);
        end
    end
end