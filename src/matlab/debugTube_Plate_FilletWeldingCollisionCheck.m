clc; clear; close all;
plotMode = 3;  
% 1 = 原始：正常蓝色 / 干涉红色 + 黄色安全
% 2 = 与当前完全一致（你原逻辑）
% 3 = 干涉只画黄色（不画红色焊枪）
%% ================== 母材数据 ==================

% 平面 Ax + By + Cz + D = 0
plane = [-0.999322, -0.0362088, -0.0066518, 960.474];

% 圆柱
C = [988.462, -131.851, -25.7489];
axis_dir = [-0.509355, 0.86045, 0.0135688];
axis_dir = axis_dir / norm(axis_dir);
cyl_radius = 56.8793;

%% ================== 焊缝轨迹 ==================

seam{1} = [
    966.057, -140.87, 25.9319, -152.51, -51.9375, -22.5674;
    967.222, -171.199, 15.975, -128.729, -48.8545, -45.0217;
    968.306, -197.957, -1.23276, -104.473, -22.6581, -68.583;
    968.945, -210.387, -29.5118, 91.9003, -23.243, -74.4308;
];

seam{2} = [
    966.057, -140.87, 25.9319, -162.027, -54.9026, -10.7349;
    964.88, -109.305, 30.8759, -175.64, -48.9587, 0.644695;
    963.719, -77.3644, 31.5746, 174.358, -41.8905, 7.77142;
    962.592, -45.6156, 28.0329, 165.314, -33.8873, 12.7102;
    961.534, -14.8505, 19.4913, 154.586, -24.2627, 16.7449;
    960.637, 12.8001, 3.72238, 138.362, -17.6232, 12.2897;
];

%% ================== 焊枪参数 ==================
tool_radius = 10;
tool_length = 100;
offset = 15;

maxExtraOffset = 300;   % 最多允许再后撤多少 mm
tolOffset = 1e-3;       % 二分搜索精度

%% ================== 画布 ==================
figure('Color','w'); hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on;

%% ================== 平面 ==================
[xp, yp] = meshgrid(900:20:1000, -250:20:50);
zp = (-plane(1)*xp - plane(2)*yp - plane(4)) / plane(3);
surf(xp, yp, zp, 'FaceAlpha',0.3,'EdgeColor','none','FaceColor','cyan');

%% ================== 圆柱 ==================
[theta, h] = meshgrid(linspace(0,2*pi,50), linspace(-200,200,50));
x = cyl_radius*cos(theta);
y = cyl_radius*sin(theta);
z = h;

tmp = [1 0 0];
if abs(dot(tmp,axis_dir)) > 0.9
    tmp = [0 1 0];
end

x_axis = cross(tmp,axis_dir); x_axis = x_axis/norm(x_axis);
y_axis = cross(axis_dir,x_axis);

R_cyl = [x_axis(:), y_axis(:), axis_dir(:)];
pts = R_cyl * [x(:)'; y(:)'; z(:)'];

Xc = reshape(pts(1,:), size(x)) + C(1);
Yc = reshape(pts(2,:), size(y)) + C(2);
Zc = reshape(pts(3,:), size(z)) + C(3);

surf(Xc, Yc, Zc,'FaceAlpha',0.2,'EdgeColor','none','FaceColor','green');

%% ================== 主循环 ==================
for s = 1:length(seam)

    poses = seam{s};
    prevZ = [];

    for i = 1:size(poses,1)

        %% ========== 当前点 ==========
        P = poses(i,1:3);

        %% ========== 欧拉角 ==========
        rx = deg2rad(poses(i,4));
        ry = deg2rad(poses(i,5));
        rz = deg2rad(poses(i,6));

        Rx = [1 0 0;
              0 cos(rx) -sin(rx);
              0 sin(rx) cos(rx)];

        Ry = [cos(ry) 0 sin(ry);
              0 1 0;
              -sin(ry) 0 cos(ry)];

        Rz = [cos(rz) -sin(rz) 0;
              sin(rz) cos(rz) 0;
              0 0 1];

        R = Rz * Ry * Rx;
        Z = R(:,3);

        %% ========== 方向约束 ==========
        if dot(Z,[0 0 1]) > 0
            R = -R;
            Z = -Z;
        end
        if ~isempty(prevZ) && dot(Z,prevZ) < 0
            R = -R;
            Z = -Z;
        end
        prevZ = Z;

        %% ========== 当前 offset 对应 base ==========
        base = P - offset * Z';

        %% =====================================================
        %%   直接在“过 base、法向为焊枪轴 Z 的平面”里计算
        %% =====================================================

        [isIntersect, dist_plane, dist_cyl, plane_hit, cyl_hit] = ...
            evalCollisionAtOffset(P, Z, offset, plane, C, axis_dir, cyl_radius, tool_radius);

        %% ================== 计算不干涉所需 offset ==================
        safeOffset = offset;
        extraOffset = 0;

        if isIntersect
            safeOffset = findSafeOffset(P, Z, offset, plane, C, axis_dir, cyl_radius, tool_radius, maxExtraOffset, tolOffset);
            if isnan(safeOffset)
                extraOffset = NaN;
            else
                extraOffset = safeOffset - offset;
            end
        end

        %% ================== 打印日志 ==================
        fprintf("\n[Seam %d | Point %d]\n", s, i);
        fprintf("  Base = [%.3f %.3f %.3f]\n", base);
        fprintf("  dist_plane   = %.4f\n", dist_plane);
        fprintf("  dist_cyl     = %.4f\n", dist_cyl);
        fprintf("  Intersect?   = %d\n", isIntersect);

        if isIntersect
            if isnan(safeOffset)
                fprintf("  safeOffset   = NOT FOUND within +%.3f mm\n", maxExtraOffset);
            else
                fprintf("  safeOffset   = %.4f\n", safeOffset);
                fprintf("  extraOffset  = %.4f\n", extraOffset);
            end
        end

        %% ================== 当前交点选择 ==================
        if isIntersect
            if isnan(dist_cyl) || (~isnan(dist_plane) && dist_plane < dist_cyl)
                hit_point = plane_hit;
            else
                hit_point = cyl_hit;
            end
        else
            hit_point = [NaN, NaN, NaN];
        end

       %% ================== 焊枪模型 / 安全焊枪 绘图控制 ==================
    [X, Y, Zm] = buildToolSurface(R, base, tool_radius, tool_length);

    drawCurrentTool = true;
    drawSafeTool = isIntersect && ~isnan(safeOffset);

    switch plotMode
        case 1
            % 干涉时当前焊枪仍按蓝色画
            currentColor = 'b';

        case 2
            % 干涉时画红色，非干涉画蓝色
            if isIntersect
                currentColor = 'r';
            else
                currentColor = 'b';
            end

        case 3
            % 干涉时只画黄色安全焊枪
            if isIntersect
                drawCurrentTool = false;
            end
            currentColor = 'b'; % 仅用于非干涉情况

        otherwise
            currentColor = 'b';
    end

    % 当前焊枪
    if drawCurrentTool
        surf(X, Y, Zm, 'FaceColor', currentColor, 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    end

    % 黄色安全焊枪
    if drawSafeTool
        safeBase = P - safeOffset * Z';
        [Xy, Yy, Zy] = buildToolSurface(R, safeBase, tool_radius, tool_length);

        surf(Xy, Yy, Zy, 'FaceColor', 'y', 'EdgeColor', 'none', 'FaceAlpha', 0.35);
        plot3(safeBase(1), safeBase(2), safeBase(3), ...
              'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 5);
    elseif isIntersect && plotMode == 3
        % mode=3 且没找到安全位姿时，兜底画红色当前焊枪
        surf(X, Y, Zm, 'FaceColor', 'r', 'EdgeColor', 'none', 'FaceAlpha', 0.6);
    end

    % 交点和连线
    if isIntersect && plotMode ~= 3
        plot3([base(1) hit_point(1)], ...
              [base(2) hit_point(2)], ...
              [base(3) hit_point(3)], ...
              'r-', 'LineWidth', 2);

        plot3(hit_point(1), hit_point(2), hit_point(3), ...
              'ro', 'MarkerFaceColor', 'r');
    end
        %% ================== 轨迹与姿态 ==================
        plot3(P(1),P(2),P(3),'ko','MarkerFaceColor','k');
        quiver3(P(1),P(2),P(3),Z(1),Z(2),Z(3),30,'k','LineWidth',1.5);

    end
end

title('焊枪轨迹 + 干涉检测 + 自动后撤 offset + 黄色安全焊枪');

%% ================== 本脚本用到的局部函数 ==================

function [X, Y, Zm] = buildToolSurface(R, base, tool_radius, tool_length)
    [th, hh] = meshgrid(linspace(0,2*pi,25), linspace(0,tool_length,25));

    x = tool_radius*cos(th);
    y = tool_radius*sin(th);
    z = -hh;

    pts = R * [x(:)'; y(:)'; z(:)'];

    X = reshape(pts(1,:), size(x)) + base(1);
    Y = reshape(pts(2,:), size(y)) + base(2);
    Zm = reshape(pts(3,:), size(z)) + base(3);
end

function [isIntersect, dist_plane, dist_cyl, plane_hit, cyl_hit] = evalCollisionAtOffset(P, Z, offset, plane, C, axis_dir, cyl_radius, tool_radius)
    base = P(:) - offset * Z(:);

    tool_n = Z(:) / norm(Z);
    [u, v] = buildPlaneBasis(tool_n);
    U = [u(:), v(:)];

    %% -------- 母材平面 --------
    n_plane = plane(1:3)';
    n_plane = n_plane / norm(n_plane);

    a_line = U' * n_plane;
    c_line = dot(n_plane, base) + plane(4);

    [dist_plane, plane_hit, ~] = closestPointOnLineInToolPlane(base, U, a_line, c_line);

    %% -------- 母材圆柱 --------
    a_cyl = axis_dir(:);
    d0 = base - C(:);

    M = eye(3) - a_cyl * a_cyl';
    A2 = U' * M * U;
    b2 = 2 * U' * M * d0;
    c2 = d0' * M * d0 - cyl_radius^2;

    [dist_cyl, cyl_hit, ~] = closestPointOnQuadraticInToolPlane(base, U, A2, b2, c2);

    %% -------- 干涉判断 --------
    isPlane = dist_plane < tool_radius;
    isCyl   = dist_cyl < tool_radius;
    isIntersect = isPlane || isCyl;

    plane_hit = plane_hit(:).';
    cyl_hit = cyl_hit(:).';
end

function safeOffset = findSafeOffset(P, Z, offset0, plane, C, axis_dir, cyl_radius, tool_radius, maxExtraOffset, tolOffset)
    [isIntersect0, ~, ~, ~, ~] = evalCollisionAtOffset(P, Z, offset0, plane, C, axis_dir, cyl_radius, tool_radius);

    if ~isIntersect0
        safeOffset = offset0;
        return;
    end

    lo = offset0;
    hi = offset0 + 1.0;

    % 先扩张上界，直到不干涉
    while hi <= offset0 + maxExtraOffset
        [isIntersectHi, ~, ~, ~, ~] = evalCollisionAtOffset(P, Z, hi, plane, C, axis_dir, cyl_radius, tool_radius);
        if ~isIntersectHi
            break;
        end
        lo = hi;
        hi = hi + max(1.0, 0.5 * (hi - offset0));
    end

    if hi > offset0 + maxExtraOffset
        safeOffset = NaN;
        return;
    end

    % 二分搜索最小 safeOffset
    for k = 1:60
        if abs(hi - lo) < tolOffset
            break;
        end

        mid = (lo + hi) / 2;
        [isIntersectMid, ~, ~, ~, ~] = evalCollisionAtOffset(P, Z, mid, plane, C, axis_dir, cyl_radius, tool_radius);

        if isIntersectMid
            lo = mid;
        else
            hi = mid;
        end
    end

    safeOffset = hi;
end

function [u, v] = buildPlaneBasis(n)
    n = n(:) / norm(n);

    tmp = [1; 0; 0];
    if abs(dot(tmp, n)) > 0.9
        tmp = [0; 1; 0];
    end

    u = cross(tmp, n);
    if norm(u) < 1e-12
        tmp = [0; 0; 1];
        u = cross(tmp, n);
    end
    u = u / norm(u);

    v = cross(n, u);
    v = v / norm(v);
end

function [distMin, hitPoint, xyBest] = closestPointOnLineInToolPlane(base, U, a, c)
    % 线：a' * x + c = 0
    % 在二维平面坐标里求距离原点最近的点
    if norm(a) < 1e-12
        distMin = abs(c);
        xyBest = [0; 0];
        hitPoint = base(:).';
        return;
    end

    xyBest = -c * a / (a' * a);
    distMin = norm(xyBest);

    hitPoint = (base(:) + U * xyBest).';
end

function [distMin, hitPoint, xyBest] = closestPointOnQuadraticInToolPlane(base, U, A, b, c)
    % 二次曲线：x' A x + b' x + c = 0
    % 通过拉格朗日乘子直接找“平面内到原点最近”的点
    A = (A + A') / 2;
    I2 = eye(2);

    lambdaCand = collectLambdaCandidates(A, b, c);

    bestDist = inf;
    xyBest = [NaN; NaN];

    for k = 1:numel(lambdaCand)
        lam = lambdaCand(k);
        H = I2 + lam * A;

        if rcond(H) < 1e-12
            continue;
        end

        xy = -(lam / 2) * (H \ b);

        if any(~isfinite(xy))
            continue;
        end

        if abs(quadraticStationaryValue(lam, A, b, c, I2)) > 1e-5
            continue;
        end

        d = norm(xy);
        if d < bestDist
            bestDist = d;
            xyBest = xy;
        end
    end

    % 兜底：如果根没收敛到，就做一次粗搜索
    if ~isfinite(bestDist)
        L = 1e5;
        grid = linspace(-L, L, 4001);
        vals = nan(size(grid));

        for i = 1:numel(grid)
            vals(i) = quadraticStationaryValue(grid(i), A, b, c, I2);
        end

        [~, idx] = min(abs(vals(isfinite(vals))));
        finiteGrid = grid(isfinite(vals));
        if ~isempty(finiteGrid)
            lam = finiteGrid(idx);
            H = I2 + lam * A;
            if rcond(H) >= 1e-12
                xyBest = -(lam / 2) * (H \ b);
                bestDist = norm(xyBest);
            end
        end
    end

    if ~isfinite(bestDist)
        distMin = NaN;
        hitPoint = [NaN, NaN, NaN];
        xyBest = [NaN; NaN];
        return;
    end

    distMin = bestDist;
    hitPoint = (base(:) + U * xyBest).';
end

function lambdaCand = collectLambdaCandidates(A, b, c)
    % 搜索驻点方程的候选 lambda
    A = (A + A') / 2;
    eigA = eig(A);
    eigA = real(eigA(abs(imag(eigA)) < 1e-10));

    poles = [];
    for i = 1:numel(eigA)
        if abs(eigA(i)) > 1e-12
            poles(end+1) = -1 / eigA(i); %#ok<AGROW>
        end
    end
    poles = sort(unique(poles));

    L = 1e5;
    edges = unique(sort([-L, poles(:).', L]));

    lambdaCand = [];

    for k = 1:numel(edges)-1
        left = edges(k);
        right = edges(k+1);

        if right - left < 1e-10
            continue;
        end

        pad = 1e-8 * max(1, abs(left) + abs(right));
        a = left + pad;
        bnd = right - pad;

        if a >= bnd
            continue;
        end

        sample = linspace(a, bnd, 101);
        vals = nan(size(sample));

        for i = 1:numel(sample)
            vals(i) = quadraticStationaryValue(sample(i), A, b, c, eye(2));
        end

        % 先收集接近零的点
        finiteMask = isfinite(vals);
        if any(finiteMask)
            finiteSample = sample(finiteMask);
            finiteVals = vals(finiteMask);

            [minAbsVal, idxMin] = min(abs(finiteVals));
            if minAbsVal < 1e-6
                lambdaCand(end+1) = finiteSample(idxMin); %#ok<AGROW>
            end
        end

        % 再收集符号变化的根
        for i = 1:numel(sample)-1
            v1 = vals(i);
            v2 = vals(i+1);

            if ~isfinite(v1) || ~isfinite(v2)
                continue;
            end

            if v1 == 0
                lambdaCand(end+1) = sample(i); %#ok<AGROW>
            elseif v1 * v2 < 0
                try
                    root = fzero(@(lam) quadraticStationaryValue(lam, A, b, c, eye(2)), ...
                                 [sample(i), sample(i+1)]);
                    lambdaCand(end+1) = root; %#ok<AGROW>
                catch
                end
            end
        end
    end

    if isempty(lambdaCand)
        lambdaCand = [];
        return;
    end

    lambdaCand = unique(round(lambdaCand * 1e10) / 1e10);
end

function val = quadraticStationaryValue(lambda, A, b, c, I2)
    H = I2 + lambda * A;

    if rcond(H) < 1e-12
        val = NaN;
        return;
    end

    x = -(lambda / 2) * (H \ b);
    if any(~isfinite(x))
        val = NaN;
        return;
    end

    val = x.' * A * x + b.' * x + c;
end