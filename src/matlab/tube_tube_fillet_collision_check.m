clc; clear; close all;

plotMode = 3;
% 1 = 原始蓝色 + 干涉红线/点 + 黄色安全
% 2 = 干涉红色 / 非干涉蓝色
% 3 = 干涉只画黄色安全焊枪

%% ================== 两个圆柱母材数据 ==================

% 圆柱1 weldCoeff
C1 = [928.696, -89.8235, -49.5261];
axis1 = [-0.438828, 0.897357, -0.0467012];
axis1 = axis1 / norm(axis1);
R1 = 57.6241;

% 圆柱2 otherSurface
C2 = [1006.25, 56.6402, -36.8684];
axis2 = [-0.0613377, -0.998019, -0.0140133];
axis2 = axis2 / norm(axis2);
R2 = 81.5506;
%% ================== 焊缝轨迹 ==================

seam{1} = [
    927.446, -70.312, 1.64969, 173.973, -26.9743, 1.92992;
    927.72, -38.8177, -0.97226, 168.853, -21.386, 3.4238;
    926.879, -7.82796, -6.43475, 162.686, -17.0534, 6.16267;
    925.586, 19.1965, -14.3606, 155.311, -14.2825, 7.57923;
    924.663, 50.3961, -27.9721, 138.191, -13.1012, 7.52018;
];

seam{2} = [
    927.446, -70.312, 1.64969, 176.61, -27.3972, -3.83744;
    925.538, -101.798, 1.25607, -174.679, -34.4646, -10.3431;
    920.766, -131.614, -4.81146, -154.165, -40.8959, -30.7657;
    914.124, -155.856, -19.347, -120.305, -34.5118, -62.2088;
    910.892, -165.947, -42.2892, -91.9595, -21.8792, -78.155;
];

%% ================== 焊枪参数 ==================

tool_radius = 10;
tool_length = 100;
offset = 15;

maxExtraOffset = 300;
tolOffset = 1e-3;

%% ================== 画布 ==================

figure('Color','w'); hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
view(3); grid on;

%% ================== 绘制两个圆柱 ==================

drawCylinder(C1, axis1, R1, 220, 'green', 0.22);
drawCylinder(C2, axis2, R2, 220, 'cyan', 0.22);

%% ================== 主循环 ==================

for s = 1:length(seam)

    poses = seam{s};

    for i = 1:size(poses,1)

        P = poses(i,1:3);

        %% ========== 直接从 abc 读取欧拉角，ZYX：R = Rz * Ry * Rx ==========
        R = eulerZYX_to_R(poses(i,4), poses(i,5), poses(i,6));
        Z = R(:,3);
        Z = Z / norm(Z);

        %% ========== 不重新算姿态，只根据圆柱1径向修正 Z 正负 ==========
        % 圆柱1径向向量：从圆柱轴线指向当前焊点
        radial1 = pointCylinderRadial(P, C1, axis1);

        % 要求 Z 与圆柱1主轴径向向量相反
        if dot(Z, radial1) > 0
            Z = -Z;
            R(:,3) = Z;

            % 为了绘制圆柱焊枪时坐标系仍大致一致，同时翻转 X
            R(:,1) = -R(:,1);
        end

        %% ========== 当前 offset 对应 base ==========
        base = P(:) - offset * Z(:);

        %% ========== 两个圆柱防碰撞 ==========
        [isIntersect, dist1, dist2, hit1, hit2] = ...
            evalCollisionTwoCylindersAtOffset(P, Z, offset, ...
                                              C1, axis1, R1, ...
                                              C2, axis2, R2, ...
                                              tool_radius);

        safeOffset = offset;
        extraOffset = 0;

        if isIntersect
            safeOffset = findSafeOffsetTwoCylinders(P, Z, offset, ...
                                                    C1, axis1, R1, ...
                                                    C2, axis2, R2, ...
                                                    tool_radius, ...
                                                    maxExtraOffset, tolOffset);
            if isnan(safeOffset)
                extraOffset = NaN;
            else
                extraOffset = safeOffset - offset;
            end
        end

        %% ========== 打印日志 ==========
        fprintf("\n[Seam %d | Point %d]\n", s, i);
        fprintf("  P    = [%.3f %.3f %.3f]\n", P(1), P(2), P(3));
        fprintf("  Z    = [%.6f %.6f %.6f]\n", Z(1), Z(2), Z(3));
        fprintf("  Base = [%.3f %.3f %.3f]\n", base(1), base(2), base(3));
        fprintf("  dist_cyl1 = %.4f\n", dist1);
        fprintf("  dist_cyl2 = %.4f\n", dist2);
        fprintf("  Intersect? = %d\n", isIntersect);

        if isIntersect
            if isnan(safeOffset)
                fprintf("  safeOffset = NOT FOUND within +%.3f mm\n", maxExtraOffset);
            else
                fprintf("  safeOffset = %.4f\n", safeOffset);
                fprintf("  extraOffset = %.4f\n", extraOffset);
            end
        end

        %% ========== 选择最近碰撞点 ==========
        if isIntersect
            if isnan(dist2) || (~isnan(dist1) && dist1 <= dist2)
                hit_point = hit1;
            else
                hit_point = hit2;
            end
        else
            hit_point = [NaN, NaN, NaN];
        end

        %% ========== 绘制当前焊枪 / 安全焊枪 ==========
        [X, Y, Zm] = buildToolSurface(R, base, tool_radius, tool_length);

        drawCurrentTool = true;
        drawSafeTool = isIntersect && ~isnan(safeOffset);

        switch plotMode
            case 1
                currentColor = 'b';

            case 2
                if isIntersect
                    currentColor = 'r';
                else
                    currentColor = 'b';
                end

            case 3
                if isIntersect
                    drawCurrentTool = false;
                end
                currentColor = 'b';

            otherwise
                currentColor = 'b';
        end

        if drawCurrentTool
            surf(X, Y, Zm, 'FaceColor', currentColor, ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.6);
        end

        if drawSafeTool
            safeBase = P(:) - safeOffset * Z(:);
            [Xy, Yy, Zy] = buildToolSurface(R, safeBase, tool_radius, tool_length);

            surf(Xy, Yy, Zy, 'FaceColor', 'y', ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.35);

            plot3(safeBase(1), safeBase(2), safeBase(3), ...
                  'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 5);

        elseif isIntersect && plotMode == 3
            surf(X, Y, Zm, 'FaceColor', 'r', ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.6);
        end

        %% ========== 干涉连线 ==========
        if isIntersect && plotMode ~= 3
            plot3([base(1) hit_point(1)], ...
                  [base(2) hit_point(2)], ...
                  [base(3) hit_point(3)], ...
                  'r-', 'LineWidth', 2);

            plot3(hit_point(1), hit_point(2), hit_point(3), ...
                  'ro', 'MarkerFaceColor', 'r');
        end

        %% ========== 轨迹点和焊枪 Z 方向 ==========
        plot3(P(1), P(2), P(3), 'ko', 'MarkerFaceColor', 'k');
        quiver3(P(1), P(2), P(3), ...
                -Z(1), -Z(2), -Z(3), ...
                30, 'k', 'LineWidth', 1.5);

    end
end

title('双圆柱母材防碰撞 + abc读取姿态 + 自动后撤 offset');

%% ================== 局部函数 ==================

function R = eulerZYX_to_R(a, b, c)
    % 对应 C++:
    % rotX = AngleAxis(a, X)
    % rotY = AngleAxis(b, Y)
    % rotZ = AngleAxis(c, Z)
    % R = rotZ * rotY * rotX

    rx = deg2rad(a);
    ry = deg2rad(b);
    rz = deg2rad(c);

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
end

function radial = pointCylinderRadial(P, C, axis_dir)
    P = P(:);
    C = C(:);
    axis_dir = axis_dir(:) / norm(axis_dir);

    v = P - C;
    foot = C + dot(v, axis_dir) * axis_dir;
    radial = P - foot;

    if norm(radial) < 1e-9
        radial = [1; 0; 0];
    else
        radial = radial / norm(radial);
    end
end

function drawCylinder(C, axis_dir, radius, halfLen, colorName, alphaVal)
    [theta, h] = meshgrid(linspace(0,2*pi,60), linspace(-halfLen, halfLen,60));

    x = radius * cos(theta);
    y = radius * sin(theta);
    z = h;

    axis_dir = axis_dir(:) / norm(axis_dir);

    tmp = [1; 0; 0];
    if abs(dot(tmp, axis_dir)) > 0.9
        tmp = [0; 1; 0];
    end

    x_axis = cross(tmp, axis_dir);
    x_axis = x_axis / norm(x_axis);

    y_axis = cross(axis_dir, x_axis);
    y_axis = y_axis / norm(y_axis);

    R_cyl = [x_axis, y_axis, axis_dir];

    pts = R_cyl * [x(:)'; y(:)'; z(:)'];

    Xc = reshape(pts(1,:), size(x)) + C(1);
    Yc = reshape(pts(2,:), size(y)) + C(2);
    Zc = reshape(pts(3,:), size(z)) + C(3);

    surf(Xc, Yc, Zc, ...
         'FaceAlpha', alphaVal, ...
         'EdgeColor', 'none', ...
         'FaceColor', colorName);
end

function [X, Y, Zm] = buildToolSurface(R, base, tool_radius, tool_length)
    [th, hh] = meshgrid(linspace(0,2*pi,25), linspace(0,tool_length,25));

    x = tool_radius * cos(th);
    y = tool_radius * sin(th);
    z = -hh;

    pts = R * [x(:)'; y(:)'; z(:)'];

    X = reshape(pts(1,:), size(x)) + base(1);
    Y = reshape(pts(2,:), size(y)) + base(2);
    Zm = reshape(pts(3,:), size(z)) + base(3);
end

function [isIntersect, dist1, dist2, hit1, hit2] = ...
    evalCollisionTwoCylindersAtOffset(P, Z, offset, ...
                                      C1, axis1, R1, ...
                                      C2, axis2, R2, ...
                                      tool_radius)

    base = P(:) - offset * Z(:);

    tool_n = Z(:) / norm(Z);
    [u, v] = buildPlaneBasis(tool_n);
    U = [u(:), v(:)];

    [dist1, hit1] = closestCylinderInToolPlane(base, U, C1, axis1, R1);
    [dist2, hit2] = closestCylinderInToolPlane(base, U, C2, axis2, R2);

    isCyl1 = isfinite(dist1) && dist1 < tool_radius;
    isCyl2 = isfinite(dist2) && dist2 < tool_radius;

    isIntersect = isCyl1 || isCyl2;

    hit1 = hit1(:).';
    hit2 = hit2(:).';
end

function [dist_cyl, cyl_hit] = closestCylinderInToolPlane(base, U, C, axis_dir, cyl_radius)
    a_cyl = axis_dir(:) / norm(axis_dir);
    d0 = base(:) - C(:);

    M = eye(3) - a_cyl * a_cyl';

    A2 = U' * M * U;
    b2 = 2 * U' * M * d0;
    c2 = d0' * M * d0 - cyl_radius^2;

    [dist_cyl, cyl_hit, ~] = closestPointOnQuadraticInToolPlane(base, U, A2, b2, c2);
end

function safeOffset = findSafeOffsetTwoCylinders(P, Z, offset0, ...
                                                 C1, axis1, R1, ...
                                                 C2, axis2, R2, ...
                                                 tool_radius, ...
                                                 maxExtraOffset, tolOffset)

    [isIntersect0, ~, ~, ~, ~] = ...
        evalCollisionTwoCylindersAtOffset(P, Z, offset0, ...
                                          C1, axis1, R1, ...
                                          C2, axis2, R2, ...
                                          tool_radius);

    if ~isIntersect0
        safeOffset = offset0;
        return;
    end

    lo = offset0;
    hi = offset0 + 1.0;

    while hi <= offset0 + maxExtraOffset
        [isIntersectHi, ~, ~, ~, ~] = ...
            evalCollisionTwoCylindersAtOffset(P, Z, hi, ...
                                              C1, axis1, R1, ...
                                              C2, axis2, R2, ...
                                              tool_radius);

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

    for k = 1:60
        if abs(hi - lo) < tolOffset
            break;
        end

        mid = (lo + hi) / 2;

        [isIntersectMid, ~, ~, ~, ~] = ...
            evalCollisionTwoCylindersAtOffset(P, Z, mid, ...
                                              C1, axis1, R1, ...
                                              C2, axis2, R2, ...
                                              tool_radius);

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

function [distMin, hitPoint, xyBest] = closestPointOnQuadraticInToolPlane(base, U, A, b, c)
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

        val = xy.' * A * xy + b.' * xy + c;
        if abs(val) > 1e-5
            continue;
        end

        d = norm(xy);
        if d < bestDist
            bestDist = d;
            xyBest = xy;
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

        finiteMask = isfinite(vals);
        if any(finiteMask)
            finiteSample = sample(finiteMask);
            finiteVals = vals(finiteMask);

            [minAbsVal, idxMin] = min(abs(finiteVals));
            if minAbsVal < 1e-6
                lambdaCand(end+1) = finiteSample(idxMin); %#ok<AGROW>
            end
        end

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