function [TrajectoryPos,TrajectoryVel,TrajectoryAcc,TrajectoryJerk]=...
    CreateTrajectory(initialState, goalState, t, Times)

for i=1:3
    waypts = [initialState(i),0;
        goalState(i),0;]';
    v0 = [initialState(i+3),0];
    a0 = [initialState(i+6),0];
    v1 = [goalState(i+3),0];
    a1 = [goalState(i+6),0];
    ts = arrangeT(waypts,t);
    n_order = 5;
    
    % trajectory plan
    polys(:,i) = minimum_snap_single_axis_close_form(waypts(1,:),ts,n_order,v0(1),a0(1),v1(1),a1(1));
end
% replace all NaNs with zeros
polys(isnan(polys))=0;
% differentiate the polynomials
vpolys=repmat([1;2;3;4;5;0],1,3).*circshift(polys,-1);
apolys=repmat([1;2;3;4;5;0],1,3).*circshift(vpolys,-1);
jpolys=repmat([1;2;3;4;5;0],1,3).*circshift(apolys,-1);

% Evaluate the polynomials
for i=1:length(Times)
    TrajectoryPos(i,:)=[Times(i) Times(i).^[0 1 2 3 4 5]*polys];
    TrajectoryVel(i,:)=[Times(i) Times(i).^[0 1 2 3 4 5]*vpolys];
    TrajectoryAcc(i,:)=[Times(i) Times(i).^[0 1 2 3 4 5]*apolys];
    TrajectoryJerk(i,:)=[Times(i) Times(i).^[0 1 2 3 4 5]*jpolys];
end