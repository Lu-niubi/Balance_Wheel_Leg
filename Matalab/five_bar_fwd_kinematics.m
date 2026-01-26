function [L0, phi0] = five_bar_fwd_kinematics(phi1, phi4)
    % 五连杆位置正解 (Forward Kinematics)
    % 输入:
    %   phi1: 左电机角度 (弧度 rad)
    %   phi4: 右电机角度 (弧度 rad)
    % 输出:
    %   L0:   虚拟腿长度 (mm)
    %   phi0: 虚拟腿角度 (弧度 rad)

    %% 1. 机械参数定义 (单位: mm)
    l1 = 61;  % 左主动臂
    l2 = 90;  % 左从动臂
    l3 = 90;  % 右从动臂 (假设对称，对应你的 l2=90 描述)
    l4 = 61;  % 右主动臂
    l5 = 38;  % 机身宽度 (两电机间距)

    %% 2. 计算主动臂末端坐标 (B点 和 D点)
    % 坐标系原点 A(0,0) 在左电机轴心
    xB = l1 * cos(phi1);
    yB = l1 * sin(phi1);

    xD = l5 + l4 * cos(phi4);
    yD = l4 * sin(phi4);

    %% 3. 构建闭环方程求解从动臂角度 (求C点)
    % 使用双圆交点法 (Double Circle Intersection)
    
    % BD 之间的距离
    lBD_sq = (xD - xB)^2 + (yD - yB)^2; % lBD squared
    lBD = sqrt(lBD_sq);

    % 构造辅助变量 (对应 C 代码中的 A0, B0, C0)
    A0 = 2 * l2 * (xD - xB);
    B0 = 2 * l2 * (yD - yB);
    C0 = l2^2 + lBD_sq - l3^2;

    % 判别式检测 (物理可行性保护)
    delta = A0^2 + B0^2 - C0^2;
    if delta < 0
        warning('目标位置物理不可达 (两腿构不成三角形)');
        L0 = NaN;
        phi0 = NaN;
        return;
    end

    % 计算 phi2 (左从动臂角度)
    % 注意：这里的 + sqrt(...) 决定了构型 (膝盖朝向)。
    % 如果发现画出来的图膝盖反了，把这里改成 - sqrt
    phi2 = 2 * atan2((B0 + sqrt(delta)), (A0 + C0));

    % 计算 C 点 (末端) 坐标
    xC = xB + l2 * cos(phi2);
    yC = yB + l2 * sin(phi2);

    %% 4. 转换为虚拟腿坐标 (L0, phi0)
    % 虚拟腿原点在机身中心 (l5/2, 0)
    x_center = xC - l5/2;
    y_center = yC;

    L0 = sqrt(x_center^2 + y_center^2);
    phi0 = atan2(y_center, x_center);
end