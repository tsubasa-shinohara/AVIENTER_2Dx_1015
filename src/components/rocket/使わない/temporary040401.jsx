export const calculateFlightPath = (rocketParams, angle, windSpeed, windProfile, config) => {
    // 設定オブジェクトからフラグを取得（設定がなければデフォルト値を使用）
    const useEnhancedAttitudeControl = config?.enhancedAttitudeControl ?? ENHANCED_ATTITUDE_CONTROL;
    const useWindAngleLimitation = config?.windAngleLimitation ?? WIND_ANGLE_LIMITATION;
    // 角度変化を記録するための変数
    let prevOmega = angle * Math.PI / 180; // 前フレームの角度（初期値は発射角度）
    let currentMaxAngleChange = 0; // 直接更新用の最大角度変化量
    const dt = 0.02;
    const dt2 = ANGLE_RESPONSE_DT; // 角度応答時間
    const mass_kg = gToKg(rocketParams.weight);
    const thrustData = MOTOR_THRUST_DATA[rocketParams.selectedMotor];
    const launchRailLength = PHYSICAL_CONSTANTS.launchRailLength; // 発射台の長さ (m)
    const MAX_TIME = 20; // 最大シミュレーション時間 (s)

    // ロケットの物理パラメータ
    const bodyDiameter = mmToM(rocketParams.bodyWidth); // ロケット直径 (m)
    const bodyRadius = bodyDiameter / 2; // ロケット半径 (m)
    const bodyLength = mmToM(rocketParams.bodyHeight + rocketParams.noseHeight); // ロケット全長 (m)
    const finwidth_m = mmToM(rocketParams.finHeight); // フィン幅
    const finThickness_m = mmToM(rocketParams.finThickness);
    const PI = Math.PI;

    // 慣性モーメントの計算 (I = 0.25*m*r^2 + 0.0833*m*l^2)
    const momentOfInertia = 0.25 * mass_kg * bodyRadius * bodyRadius + 0.0833 * mass_kg * bodyLength * bodyLength;

    // ノーズ形状に基づく抗力係数
    const noseCd = NOSE_SHAPES[rocketParams.noseShape].cd;

    // フィン材料特性
    const finMaterial = FIN_MATERIALS[rocketParams.finMaterial];

    // パラシュート関連の定数
    const thrustEndTime = thrustData.length * dt;
    const parachuteDelay = parseInt(rocketParams.selectedMotor.split('-')[1]);
    const parachuteDeployTime = 1.0;
    const parachuteEjectionTime = thrustEndTime + parachuteDelay;
    const parachuteActiveTime = parachuteEjectionTime + parachuteDeployTime;
    const parachuteDiameter = mmToM(parseInt(rocketParams.selectedParachute.slice(1))); // φ180 -> 180mm -> 0.18m

    // 新しい計算 - 投影面積と圧力中心の計算
    const projectedAreas = calculateProjectedArea(rocketParams);
    const centerOfPressure = calculateCenterOfPressure(rocketParams);
    const aerodynamicCenter = calculateAerodynamicCenter(rocketParams);
    const stabilityCenterOfPressure = calculateStabilityCenterOfPressure(rocketParams);

    rocketParams.finCp = centerOfPressure.finCp;

    // 体積計算
    const volumes = calculateVolume(rocketParams);

    // 静安定マージン計算
    const staticMargins = calculateStaticMargin(rocketParams);

    // 状態変数の初期化
    let time = 0;
    let x = 0; // メートル単位
    let y = 0; // メートル単位
    let vx = 0;
    let vy = 0;
    let prev_vx = 0;
    let prev_vy = 0;
    let omega = angle * Math.PI / 180; // 初期角度（ラジアン）
    let angularVelocity = 0; // 角速度
    let angularAcceleration = 0; // 角加速度
    let data = [];
    let isParachuteEjected = false;
    let isParachuteActive = false;
    let parachuteDeploymentProgress = 0;
    let finDeflection = 0; // フィンのたわみ量（mm）

    // 姿勢安定性チェック用の変数を追加
    let maxAngleChangePerDt2 = 0; // dt2時間あたりの最大角度変化量
    let isAngleStableOK = true; // 姿勢安定性の判定
    let isAbsoluteAngleOK = true; // 絶対角度の判定
    const MAX_ANGLE_CHANGE_PER_DT2 = 45.0; // 角度変化量の最大値（±45度）- 変更点1: 10°→45°
    const MAX_ABSOLUTE_ANGLE = 112.5; // 絶対角度の最大値（±112.5度）- 変更点2: 新規追加
    let thrustEndFlag = false; // 推力終了フラグ
    let angleChanges = []; // 角度変化履歴を記録
    const angleChangeBuffer = []; // dt2時間ごとの角度変化を記録するバッファ
    const initialOmegaDegrees = angle; // 初期角度（度）
    let maxAbsoluteAngle = 0; // 最大絶対角度の記録用 - 変更点3: 新規追加

    // 角度計算用変数
    let stepCounter = 0;
    let totalTorque = 0;
    let avgThrustForTorque = 0;
    let thrustSamplesCount = 0;
    let angleChangePerDt2 = 0; // dt2時間あたりの角度変化量 (ν)

    // 重力加速度
    const g = 9.81; // m/s²

    // 記録用変数
    let maxHeight = 0;
    let maxSpeed = 0;
    let maxDistance = 0; // 最大水平距離を記録
    let maxFinDeflection = 0; // 最大フィンたわみ量

    // キーポイント記録
    let keyPoints = {
        thrustEnd: { time: 0, height: 0, speed: 0 },
        maxHeight: { time: 0, height: 0, speed: 0 },
        parachuteEjection: { time: 0, height: 0, speed: 0 },
        parachuteActive: { time: 0, height: 0, speed: 0 }
    };

    // シミュレーションループ
    while ((y >= 0 || time < 0.1) && time < MAX_TIME) {
        // 前回の速度を保存
        prev_vx = vx;
        prev_vy = vy;

        // パラシュート状態の更新
        if (!isParachuteEjected && time >= parachuteEjectionTime) {
            isParachuteEjected = true;
            keyPoints.parachuteEjection = { time, height: y, speed: vy };
        }

        if (isParachuteEjected && !isParachuteActive) {
            parachuteDeploymentProgress = Math.min(1,
                (time - parachuteEjectionTime) / parachuteDeployTime);
        }

        if (!isParachuteActive && time >= parachuteActiveTime) {
            isParachuteActive = true;
            parachuteDeploymentProgress = 1.0;

            // パラシュート展開時は速度を90%減少
            vx = vx * 0.1;
            vy = vy * 0.1;

            keyPoints.parachuteActive = { time, height: y, speed: vy };
        }

        const distanceFromStart = Math.sqrt(x * x + y * y);
        const onLaunchRail = distanceFromStart < launchRailLength;

        // 速度の大きさ
        const velocity = Math.sqrt(prev_vx * prev_vx + prev_vy * prev_vy);

        // 現在の高度に基づく有効風速を計算
        const effectiveWindSpeed = calculateWindSpeedAtHeight(windSpeed, y, windProfile);

        // 力と加速度の初期化
        let ax = 0;
        let ay = -g;
        let torque = 0; // トルク
        let Fx = 0;
        let Fy = 0;
        let thrust = 0;

        // 特定の角度への対応
        const angleAdjustment = Math.abs(angle) === 4 || Math.abs(angle) === 18 ? 0.01 : 0;
        const adjustedOmega = omega + angleAdjustment * (angle < 0 ? -1 : 1);

        // 絶対角度の計算と判定 - 変更点4: 絶対角度の計算と判定を追加
        const absoluteOmegaDegrees = (adjustedOmega * 180 / Math.PI) % 360;
        const normalizedAbsoluteAngle = absoluteOmegaDegrees > 180 ? absoluteOmegaDegrees - 360 : absoluteOmegaDegrees;

        // 最大絶対角度を更新
        if (Math.abs(normalizedAbsoluteAngle) > Math.abs(maxAbsoluteAngle)) {
            maxAbsoluteAngle = normalizedAbsoluteAngle;
        }

        // 絶対角度が閾値を超えたら安定性NG判定
        if (Math.abs(normalizedAbsoluteAngle) > MAX_ABSOLUTE_ANGLE) {
            isAbsoluteAngleOK = false;
            console.log(`絶対角度の閾値超過: ${normalizedAbsoluteAngle.toFixed(2)}° > ±${MAX_ABSOLUTE_ANGLE}° (t=${time.toFixed(2)}s)`);
        }

        // フィンのたわみ量計算
        if (velocity > 5.0) {
            const finParams = {
                finHeight: rocketParams.finHeight,
                finBaseWidth: rocketParams.finBaseWidth,
                finTipWidth: rocketParams.finTipWidth,
                finThickness: rocketParams.finThickness,
                finSweepLength: rocketParams.finSweepLength
            };

            finDeflection = calculateFinDeflection(velocity, finMaterial, finParams, angleChangePerDt2);

            // 最大フィンたわみ量を更新
            if (finDeflection > maxFinDeflection) {
                maxFinDeflection = finDeflection;
            }
        } else {
            finDeflection = 0;
        }

        // パラシュートフェーズ
        if (isParachuteActive) {
            // パラシュートの抗力計算
            const Cd = 0.775; // パラシュートの抗力係数
            const rho = 1.225; // 空気密度 (kg/m³)
            const Area = PI * Math.pow(parachuteDiameter / 2, 2);
            const Dp = 0.5 * Cd * rho * velocity * velocity * Area;

            // 速度方向への抗力
            if (velocity > 0.001) {
                Fx = -Dp * (prev_vx / velocity);
                Fy = -Dp * (prev_vy / velocity);
            }

            // 横風の影響を追加（高度に応じた風速を使用）
            const Cdw = 0.25; // 横風の抗力係数
            const S = parachuteDiameter * parachuteDiameter * 0.785; // パラシュートの投影面積
            const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
            Fx -= Dw; // 横風の影響を追加

            // 重力の追加
            Fy -= mass_kg * g;

            // 加速度計算
            ax = Fx / mass_kg;
            ay = Fy / mass_kg;

            // トルク計算 - パラシュート展開後は発射角度を維持する
            const initialOmega = angle * Math.PI / 180;
            if (Math.abs(adjustedOmega - initialOmega) > 0.01) {
                // 簡易的なトルク - 後でより正確な計算に置き換え
                torque = (initialOmega - adjustedOmega) * 0.001;
            }
        }
        else if (isParachuteEjected && !isParachuteActive) {
            // パラシュート展開中
            Fy = -mass_kg * g;

            // 軽い空気抵抗
            if (velocity > 0.001) {
                const dragCoefficient = 0.1;
                Fx = -dragCoefficient * prev_vx;
                Fy -= dragCoefficient * prev_vy;
            }

            // 横風の影響を小さく追加（高度に応じた風速を使用）
            const rho = 1.225; // 空気密度
            const Cdw = 0.25; // 横風の抗力係数
            const S = bodyDiameter * bodyLength * 0.5; // 半分展開時の面積
            const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
            Fx -= Dw * 0.5; // 展開中なので横風の影響を半分に

            // 加速度計算
            ax = Fx / mass_kg;
            ay = Fy / mass_kg;

            // トルク計算 - パラシュート展開中も発射角度に強く引っ張られる
            const initialOmega = angle * Math.PI / 180;
            // 簡易的なトルク - 後でより正確な計算に置き換え
            torque = (initialOmega - adjustedOmega) * 0.0005;
        }
        else {
            // 通常飛行フェーズ

            // ロケットの抗力計算 - ノーズ形状に基づく抗力係数を使用
            const Cd = noseCd;
            const rho = 1.225; // 空気密度
            const Area = PI * Math.pow(bodyDiameter / 2, 2) + finwidth_m * finThickness_m * 4;
            const Dt = 0.5 * Cd * rho * velocity * velocity * Area;

            // 横風の抗力計算（高度に応じた風速を使用）
            const Cdw = 0.25; // 横風の抗力係数
            const S = bodyDiameter * bodyLength; // ロケットの側面積
            const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;

            if (time < thrustEndTime) {
                // エンジン推力フェーズ
                const thrustIndex = Math.min(Math.floor(time / dt), thrustData.length - 1);
                thrust = thrustData[thrustIndex];

                // 推力サンプルをトルク計算用に累積
                avgThrustForTorque += thrust;
                thrustSamplesCount++;

                if (onLaunchRail) {
                    // 発射台上での運動
                    if (angle === 0) {
                        // 垂直発射
                        Fy = thrust - mass_kg * g;
                        Fx = -Dw; // 横風の影響のみ
                    } else {
                        // 角度付き発射 (修正した式)
                        Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g;
                        Fx = thrust * Math.sin(adjustedOmega) - Dw; // 横風の影響を追加
                    }

                    // 発射台上は角度固定
                    torque = 0;
                } else {
                    // 自由飛行（推力あり）
                    if (velocity > 0.001) {
                        // 修正: 推力と抗力の分解方法を修正
                        // T*sinθ - Dt*sinθ - Dw, T*cosθ - m*g - Dt*cosθ
                        Fx = thrust * Math.sin(adjustedOmega) - Dt * Math.sin(adjustedOmega) - Dw;
                        Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g - Dt * Math.cos(adjustedOmega);
                    } else {
                        // 速度がほぼゼロの場合
                        Fx = thrust * Math.sin(adjustedOmega) - Dw;
                        Fy = thrust * Math.cos(adjustedOmega) - mass_kg * g;
                    }

                    // トルク計算 - 推力フェーズ
                    if (velocity > 1.0) {
                        const flightAngle = Math.atan2(prev_vx, prev_vy);

                        try {
                            // 新しいモーメント計算関数を使用
                            const ML = calculateLiftMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
                            const MD = calculateDragMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
                            const MW = calculateWindMoment(rocketParams.noseHeight, bodyDiameter, rocketParams.bodyHeight, effectiveWindSpeed, adjustedOmega, projectedAreas.totalFinArea, centerOfPressure.centerOfPressure, rocketParams.centerOfGravity);
                            const MF = calculateFinMoment(rocketParams.finHeight, rocketParams.finBaseWidth, rocketParams.finCount, velocity, flightAngle, rocketParams, rocketParams.finCp, rocketParams.centerOfGravity);

                            // トルク値の検証 - 無限大や非数値をチェック
                            if (!isFinite(ML) || isNaN(ML)) torque += 0;
                            else if (!isFinite(MD) || isNaN(MD)) torque += 0;
                            else if (!isFinite(MW) || isNaN(MW)) torque += 0;
                            else if (!isFinite(MF) || isNaN(MF)) torque += 0;
                            else {
                                // 合計トルク - 上限設定を追加
                                const rawTorque = ML + MD + MW + MF;

                                // トルク値が小さすぎる問題を解決するために、最小トルク閾値を設定
                                const MIN_TORQUE_THRESHOLD = 0.0001;

                                // トルクの絶対値が閾値以下の場合、符号を保持して最小値を使用
                                if (Math.abs(rawTorque) > 0 && Math.abs(rawTorque) < MIN_TORQUE_THRESHOLD) {
                                    torque = Math.sign(rawTorque) * MIN_TORQUE_THRESHOLD;
                                } else {
                                    // 通常のトルク制限
                                    torque = Math.max(-1.0, Math.min(1.0, rawTorque));
                                }

                                // デバッグ用ログ - トルク計算の詳細を表示
                                if (Math.abs(torque) > 0.001 || Math.abs(ML) > 0.001 || Math.abs(MD) > 0.001 || Math.abs(MW) > 0.001 || Math.abs(MF) > 0.001) {
                                    console.log(`Thrust Torque components (t=${time.toFixed(2)}): ML=${ML.toFixed(6)}, MD=${MD.toFixed(6)}, MW=${MW.toFixed(6)}, MF=${MF.toFixed(6)}, Total=${torque.toFixed(6)}`);
                                }
                            }

                            // ±4°と±18°の場合のトルク補正
                            if (Math.abs(angle) === 4 || Math.abs(angle) === 18) {
                                torque *= 1.2; // 20%増加
                            }
                        } catch (error) {
                            // エラーが発生した場合はトルクをゼロにして続行
                            console.error('Torque calculation error:', error);
                            torque = 0;
                        }
                    }
                }
            } else {
                // 推力終了時のフラグを設定
                if (!thrustEndFlag) {
                    thrustEndFlag = true;
                    keyPoints.thrustEnd = { time, height: y, speed: vy };
                }

                // 慣性飛行（推力なし）- ここでは推力T=0

                if (velocity > 0.001) {
                    // 修正: Fx = -Dt*sinθ - Dw, Fy = -m*g - Dt*cosθ
                    Fx = -Dt * Math.sin(adjustedOmega) - Dw;
                    Fy = -mass_kg * g - Dt * Math.cos(adjustedOmega);
                } else {
                    Fx = -Dw;
                    Fy = -mass_kg * g;
                }

                // トルク計算 - 慣性飛行フェーズ
                if (velocity > 0.5) {
                    const flightAngle = Math.atan2(prev_vx, prev_vy);

                    // 新しいモーメント計算メソッドを使用
                    try {
                        // 新しいモーメント計算関数を使用
                        const ML = calculateLiftMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
                        const MD = calculateDragMoment(velocity, adjustedOmega, flightAngle, rocketParams, projectedAreas.sideArea, aerodynamicCenter.aerodynamicCenter, rocketParams.centerOfGravity);
                        const MW = calculateWindMoment(rocketParams.noseHeight, bodyDiameter, rocketParams.bodyHeight, effectiveWindSpeed, adjustedOmega, projectedAreas.totalFinArea, centerOfPressure.centerOfPressure, rocketParams.centerOfGravity);
                        const MF = calculateFinMoment(rocketParams.finHeight, rocketParams.finBaseWidth, rocketParams.finCount, velocity, flightAngle, rocketParams, rocketParams.finCp, rocketParams.centerOfGravity);

                        // 合計トルク
                        const rawTorque = ML + MD + MW + MF;

                        // トルク値が小さすぎる問題を解決するために、最小トルク閾値を設定
                        const MIN_TORQUE_THRESHOLD = 0.0001;

                        // トルクの絶対値が閾値以下の場合、符号を保持して最小値を使用
                        if (Math.abs(rawTorque) > 0 && Math.abs(rawTorque) < MIN_TORQUE_THRESHOLD) {
                            torque = Math.sign(rawTorque) * MIN_TORQUE_THRESHOLD;
                        } else {
                            // 通常のトルク制限
                            torque = Math.max(-1.0, Math.min(1.0, rawTorque));
                        }

                        // デバッグ用ログ - トルク計算の詳細を表示
                        if (Math.abs(torque) > 0.001 || Math.abs(ML) > 0.001 || Math.abs(MD) > 0.001 || Math.abs(MW) > 0.001 || Math.abs(MF) > 0.001) {
                            console.log(`Inertial Torque components (t=${time.toFixed(2)}): ML=${ML.toFixed(6)}, MD=${MD.toFixed(6)}, MW=${MW.toFixed(6)}, MF=${MF.toFixed(6)}, Total=${torque.toFixed(6)}`);
                        }
                    } catch (error) {
                        console.error('Torque calculation error:', error);
                        torque = 0;
                    }

                    // ±4°と±18°の場合のトルク補正
                    if (Math.abs(angle) === 4 || Math.abs(angle) === 18) {
                        torque *= 1.2; // 20%増加
                    }
                }
            }

            // 加速度計算
            ax = Fx / mass_kg;
            ay = Fy / mass_kg;
        }

        // トルクの累積
        totalTorque += torque;
        stepCounter++;

        // 角度の更新（dt2間隔で）
        if (stepCounter >= ANGLE_STEPS_PER_UPDATE) {
            try {
                // トルクの平均を計算
                const avgTorque = totalTorque / stepCounter;

                // トルクが非数値または無限大の場合はゼロにリセット
                const safeAvgTorque = isFinite(avgTorque) && !isNaN(avgTorque) ? avgTorque : 0;

                // 最小トルク保証 - これにより姿勢変化が確実に発生する
                const MIN_TORQUE = 0.00001;
                let effectiveTorque = safeAvgTorque;
                if (Math.abs(safeAvgTorque) > 0 && Math.abs(safeAvgTorque) < MIN_TORQUE) {
                    effectiveTorque = Math.sign(safeAvgTorque) * MIN_TORQUE;
                }

                // トルク計算のデバッグ出力
                if (Math.abs(effectiveTorque) > 0.001) {
                    console.log(`Torque calculation (t=${time.toFixed(2)}): avgTorque=${safeAvgTorque.toFixed(6)}, effectiveTorque=${effectiveTorque.toFixed(6)}, momentOfInertia=${momentOfInertia.toFixed(6)}`);
                }

                // 角加速度の計算: α = M/I
                angularAcceleration = effectiveTorque / momentOfInertia;

                // 角速度の更新: ω = ω0 + α*dt2
                const oldAngularVelocity = angularVelocity;
                angularVelocity = angularVelocity + angularAcceleration * dt2;

                // 角速度に上限を設定
                angularVelocity = Math.max(-3, Math.min(3, angularVelocity));

                // 角度変化の計算: ν = ω*dt2
                angleChangePerDt2 = angularVelocity * dt2;

                // 最小角度変化保証 - 非常に小さな値でも角度更新を保証
                const MIN_ANGLE_CHANGE_PER_DT2 = 0.001; // 角度変化を強調するために増加（0.0001→0.001）

                if (Math.abs(angleChangePerDt2) > 0 && Math.abs(angleChangePerDt2) < MIN_ANGLE_CHANGE_PER_DT2) {
                    angleChangePerDt2 = Math.sign(angleChangePerDt2) * MIN_ANGLE_CHANGE_PER_DT2;
                }

                // 角度変化のデバッグ出力
                if (Math.abs(oldAngularVelocity - angularVelocity) > 0.001 || Math.abs(angleChangePerDt2) > 0.001) {
                    console.log(`Angle change calculation (t=${time.toFixed(2)}): angularAcceleration=${angularAcceleration.toFixed(6)}, angularVelocity=${angularVelocity.toFixed(6)}, angleChangePerDt2=${angleChangePerDt2.toFixed(6)}`);
                }

                // 姿勢安定性チェック - 推力フェーズと慣性飛行フェーズの両方でチェック（パラシュート展開前のみ）
                // 変更点5: 推力フェーズも含めて姿勢安定性をチェック
                if (!isParachuteEjected) {
                    // dt2時間あたりの角度変化量（度数法）
                    const angleChangePerDt2Degrees = angleChangePerDt2 * 180 / Math.PI;

                    // 最大角度変化量を更新
                    if (Math.abs(angleChangePerDt2Degrees) > Math.abs(maxAngleChangePerDt2)) {
                        maxAngleChangePerDt2 = angleChangePerDt2Degrees;
                        console.log(`新しい最大角度変化量検出: ${angleChangePerDt2Degrees.toFixed(2)}° (t=${time.toFixed(2)}s, 推力フェーズ=${time < thrustEndTime})`);
                    }

                    // 角度変化量が閾値（±45°）を超える場合にNG判定（0.2秒間の変化量）
                    // 変更点6: 閾値を±10°から±45°に変更
                    if (Math.abs(angleChangePerDt2Degrees) > MAX_ANGLE_CHANGE_PER_DT2) {
                        isAngleStableOK = false;
                        console.log(`角度変化量の閾値超過: ${angleChangePerDt2Degrees.toFixed(2)}° > ±${MAX_ANGLE_CHANGE_PER_DT2}° (t=${time.toFixed(2)}s, 推力フェーズ=${time < thrustEndTime})`);
                    }
                }
                // カウンターとトルク累積のリセット
                stepCounter = 0;
                totalTorque = 0;
                avgThrustForTorque = 0;
                thrustSamplesCount = 0;
            } catch (error) {
                // エラーが発生した場合でも安全に処理
                console.error('Angle update error:', error);
                stepCounter = 0;
                totalTorque = 0;
                avgThrustForTorque = 0;
                thrustSamplesCount = 0;
            }
        }

        // 物理ベースの角度更新を強化 - この部分は常に適用される
        // 角度の更新 - angleChangePerDt2の値を確実に反映
        const physicsBasedAngleChange = angleChangePerDt2 / ANGLE_STEPS_PER_UPDATE;

        // 物理ベースの変化が非常に小さい場合の最小変化量を設定
        const MIN_ANGLE_CHANGE = 0.0001; // ラジアン単位での最小角度変化

        // 物理ベースの姿勢変化を適用
        if (PHYSICAL_ATTITUDE_CONTROL) {
            if (Math.abs(physicsBasedAngleChange) > 0 && Math.abs(physicsBasedAngleChange) < MIN_ANGLE_CHANGE) {
                // 変化量が小さすぎる場合は適切な最小値を使用
                omega += Math.sign(physicsBasedAngleChange) * MIN_ANGLE_CHANGE;
            } else {
                // 通常の物理ベースの角度更新
                omega += physicsBasedAngleChange;
            }

            // 角度変化のデバッグ出力 - 実際に何が起きているかを確認
            if (Math.abs(physicsBasedAngleChange) > 0.001) {
                console.log(`Applied physics angle change (t=${time.toFixed(2)}): ${physicsBasedAngleChange.toFixed(6)}, new omega=${omega.toFixed(6)}`);
            }

            // 前回の角度と現在の角度から変化量を計算（度数法に変換）
            const prevOmegaDegrees = (prevOmega * 180 / Math.PI) % 360;
            const currentOmegaDegrees = (omega * 180 / Math.PI) % 360;

            // 角度の差を-180°から180°の範囲に正規化（ラップアラウンド対応）
            let deltaOmega = currentOmegaDegrees - prevOmegaDegrees;
            if (deltaOmega > 180) deltaOmega -= 360;
            if (deltaOmega < -180) deltaOmega += 360;

            // 角度変化履歴を記録
            angleChanges.push({
                time,
                deltaOmega,
                thrustActive: time <= thrustEndTime,
                parachuteActive: isParachuteActive
            });

            // 角度変化バッファに追加
            angleChangeBuffer.push(deltaOmega);

            // バッファが0.2秒分（dt2/dt フレーム数）以上になったら古いデータを削除
            const framesPerDt2 = ANGLE_RESPONSE_DT / dt;
            if (angleChangeBuffer.length > framesPerDt2) {
                angleChangeBuffer.shift();
            }

            // バッファ内の合計角度変化量を計算
            const totalAngleChange = angleChangeBuffer.reduce((sum, change) => sum + change, 0);

            // 姿勢安定性チェック（パラシュート展開前かつ推力終了後の慣性飛行中）
            if (!isParachuteEjected && thrustEndFlag) {
                // 最大角度変化量を更新
                if (Math.abs(totalAngleChange) > Math.abs(maxAngleChangePerDt2)) {
                    maxAngleChangePerDt2 = totalAngleChange;

                    // デバッグ用ログ出力
                    console.log(`新しい最大角度変化量検出: ${totalAngleChange.toFixed(2)}° (t=${time.toFixed(2)}s)`);
                }

                // 角度変化量が±10°を超える場合にNG判定（0.2秒間の変化量）
                if (Math.abs(totalAngleChange) > 10) {
                    isAngleStableOK = false;
                }
            }

            // 前回の角度を更新
            prevOmega = omega;
        }

        // 拡張姿勢制御ロジック - 風見効果など
        if (useEnhancedAttitudeControl) {
            // 発射台を離れた後かつパラシュート展開前
            if (!onLaunchRail && !isParachuteEjected) {
                // 速度ベクトルの方向（飛行角）
                const flightAngle = Math.atan2(vx, vy);

                // 推力フェーズと慣性飛行フェーズで完全に分ける
                if (thrustEndFlag) {
                    //===========================================
                    // 慣性飛行中 - 風見効果を無効化
                    //===========================================

                    // 目標姿勢角は常に速度方向（風の影響を受けない）
                    const targetOmega = flightAngle;

                    // 姿勢角変化のスピード係数 - 非常に小さな値
                    const adjustRate = Math.min(0.05, 0.002 * velocity);

                    // 姿勢角を目標角に近づける（緩やかに）
                    const newOmega = omega * (1.0 - adjustRate) + targetOmega * adjustRate;

                    // 変化量の制限も小さめ
                    // 変化量の制限を大きくする - 約11.5度/フレーム
                    const maxChange = 0.2;
                    if (Math.abs(newOmega - omega) > maxChange) {
                        omega += Math.sign(newOmega - omega) * maxChange;
                    } else {
                        omega = newOmega;
                    }

                    // デバッグ用ログ - 拡張姿勢制御の適用を確認
                    if (Math.abs(newOmega - omega) > 0.01) {
                        console.log(`Applied enhanced attitude control (inertial): targetOmega=${targetOmega.toFixed(6)}, newOmega=${newOmega.toFixed(6)}`);
                    }

                } else {
                    //===========================================
                    // 推力飛行中 - 風見効果を適用
                    //===========================================

                    // 風向きベクトル
                    const windAngle = effectiveWindSpeed > 0 ? 0 : Math.PI; // 風が左から右:0度、右から左:180度

                    // 風向きに対する垂直方向 (±90度)
                    const perpendicularToWind1 = windAngle + Math.PI / 2;  // +90度
                    const perpendicularToWind2 = windAngle - Math.PI / 2;  // -90度

                    // 目標姿勢角の決定
                    let targetOmega;

                    if (Math.abs(effectiveWindSpeed) < 0.5) {
                        // 風が弱い場合は速度方向に合わせる
                        targetOmega = flightAngle;
                    } else {
                        // 風が強い場合、風向きも考慮する

                        // 風上に向かっているか判定 - 風向きと速度が「逆」のとき風上
                        const isMovingUpwind = (effectiveWindSpeed < 0 && vx > 0) ||
                            (effectiveWindSpeed > 0 && vx < 0);

                        if (isMovingUpwind && useWindAngleLimitation) {
                            // 風上に向かう場合は風に対して垂直方向を超えない（風向きによる90度制限が有効な場合）
                            if (effectiveWindSpeed < 0) {
                                // 右からの風の場合、-90度まで（右向き）
                                targetOmega = Math.max(flightAngle, perpendicularToWind2);
                            } else {
                                // 左からの風の場合、+90度まで（左向き）
                                targetOmega = Math.min(flightAngle, perpendicularToWind1);
                            }
                        } else {
                            // 風下に向かう場合または風向きによる角度制限が無効な場合は直接速度方向
                            targetOmega = flightAngle;
                        }
                    }

                    // 推力飛行中の姿勢角変化のスピード係数（現状維持）
                    const adjustRate = Math.min(0.05, 0.002 * velocity);

                    // 風速に応じた調整
                    const windFactor = Math.min(0.8, Math.abs(effectiveWindSpeed) / 6.0);
                    const finalAdjustRate = adjustRate * (1.0 + windFactor);

                    // 姿勢角を目標角に近づける
                    const newOmega = omega * (1.0 - finalAdjustRate) + targetOmega * finalAdjustRate;

                    // 変化量の制限を大きくする - 約11.5度/フレーム
                    const maxChange = 0.2;
                    if (Math.abs(newOmega - omega) > maxChange) {
                        omega += Math.sign(newOmega - omega) * maxChange;
                    } else {
                        omega = newOmega;
                    }

                    // デバッグ用ログ - 拡張姿勢制御の適用を確認
                    if (Math.abs(newOmega - omega) > 0.01) {
                        console.log(`Applied enhanced attitude control (thrust): targetOmega=${targetOmega.toFixed(6)}, newOmega=${newOmega.toFixed(6)}`);
                    }
                }
            }
        }

        // 速度の更新
        vx = vx + ax * dt;
        vy = vy + ay * dt;

        // 速度制限（最大100m/s）
        const MAX_SPEED = 100; // m/s
        const currentSpeed = Math.sqrt(vx * vx + vy * vy);
        if (currentSpeed > MAX_SPEED) {
            const factor = MAX_SPEED / currentSpeed;
            vx *= factor;
            vy *= factor;
        }

        // 角速度の制限
        const MAX_ANGULAR_VELOCITY = 5; // rad/s
        angularVelocity = Math.max(-MAX_ANGULAR_VELOCITY, Math.min(MAX_ANGULAR_VELOCITY, angularVelocity));

        // 位置の更新
        if (onLaunchRail) {
            // 発射台上の動き
            if (angle === 0) {
                // 垂直発射
                x = 0;
                y = y + vy * dt;
            } else {
                // 角度付き発射
                const railDistance = Math.min(distanceFromStart + velocity * dt, launchRailLength);
                x = railDistance * Math.sin(omega);
                y = railDistance * Math.cos(omega);
            }
        } else {
            // 自由飛行の位置更新
            x = x + vx * dt;
            y = y + vy * dt;
        }

        // 最高高度と最高速度の更新
        if (y > maxHeight) {
            maxHeight = y;
            keyPoints.maxHeight = { time, height: maxHeight, speed: vy };
        }

        if (currentSpeed > maxSpeed) {
            maxSpeed = currentSpeed;
        }

        // 最大水平距離の更新
        if (Math.abs(x) > maxDistance) {
            maxDistance = Math.abs(x);
        }

        // バッファ内の合計角度変化量を計算
        const totalAngleChange = angleChangeBuffer.length > 0 ?
            angleChangeBuffer.reduce((sum, change) => sum + change, 0) : 0;

        // データの記録 - 角度変化情報を追加
        data.push({
            time,
            physicsX: x, // 物理座標系でのx（メートル単位）
            physicsY: y, // 物理座標系でのy（メートル単位）
            height: y, // メートル単位
            vx: vx,
            vy: vy,
            ax: ax,
            ay: ay,
            speedMagnitude: Math.sqrt(vx * vx + vy * vy),
            accelerationMagnitude: Math.sqrt(ax * ax + ay * ay),
            angularVelocity,
            angularAcceleration,
            isParachuteEjected,
            isParachuteActive,
            parachuteDeploymentProgress,
            omega,
            omegaDegrees: (omega * 180 / Math.PI), // 角度を度数法で保存
            torque,
            angleChangePerDt2: totalAngleChange, // 現在の0.2秒間の角度変化
            horizontalDistance: Math.abs(x), // 水平距離の絶対値を追加
            finDeflection, // フィンのたわみ量を追加
            angleDeviationDegrees: (omega * 180 / Math.PI) - initialOmegaDegrees, // 初期角度からの偏差を追加
            effectiveWindSpeed, // 実効風速を記録
            isThrustActive: time <= thrustEndTime, // 推力が有効かどうか
            absoluteAngleDegrees: normalizedAbsoluteAngle, // 絶対角度を追加
            isAbsoluteAngleOK, // 絶対角度の判定結果
            angleChangeLimit: MAX_ANGLE_CHANGE_PER_DT2, // 角度変化量の閾値
            absoluteAngleLimit: MAX_ABSOLUTE_ANGLE, // 絶対角度の閾値
        });

        time += dt;
    }

    // シミュレーション終了時に角度安定性の判定結果をログ出力
    console.log(`シミュレーション完了: 最高高度=${maxHeight.toFixed(2)}m, 最高速度=${maxSpeed.toFixed(2)}m/s, 最大水平距離=${maxDistance.toFixed(2)}m`);
    console.log(`推力終了時 (${keyPoints.thrustEnd.time.toFixed(2)}s): 高度=${keyPoints.thrustEnd.height.toFixed(2)}m, 速度=${keyPoints.thrustEnd.speed.toFixed(2)}m/s`);
    console.log(`最高点 (${keyPoints.maxHeight.time.toFixed(2)}s): 高度=${keyPoints.maxHeight.height.toFixed(2)}m, 速度=${keyPoints.maxHeight.speed.toFixed(2)}m/s`);
    console.log(`最大フィンたわみ量: ${maxFinDeflection.toFixed(4)}mm`);
    console.log(`最大角度変化量/dt2: ${maxAngleChangePerDt2.toFixed(2)}°`);
    console.log(`姿勢安定性判定: ${isAngleStableOK ? 'OK' : 'NG'}`);

    // 投影面積と体積の情報を追加
    console.log(`投影面積 - 正面: ${projectedAreas.frontalArea.toFixed(5)}m², 側面: ${projectedAreas.sideArea.toFixed(5)}m²`);
    console.log(`体積 - ノーズ: ${volumes.noseVolume.toFixed(6)}m³, ボディ: ${volumes.bodyVolume.toFixed(6)}m³, 合計: ${volumes.totalVolume.toFixed(6)}m³`);
    console.log(`圧力中心位置: ${centerOfPressure.centerOfPressure.toFixed(2)}mm, 空力中心位置: ${aerodynamicCenter.aerodynamicCenter.toFixed(2)}mm`);
    console.log(`静安定用圧力中心位置: ${stabilityCenterOfPressure.stabilityCenterOfPressure.toFixed(2)}mm`);
    console.log(`静安定マージン (標準): ${staticMargins.standardStaticMargin.toFixed(2)}, (静安定用): ${staticMargins.stabilityStaticMargin.toFixed(2)}`);

    // 姿勢安定性の詳細情報を出力
    console.log("姿勢安定性の詳細情報:");
    console.log(`  - 最大角度変化量: ${maxAngleChangePerDt2.toFixed(2)}°/0.2秒`);
    console.log(`  - 姿勢安定性判定: ${isAngleStableOK ? 'OK' : 'NG'}`);
    console.log(`  - 判定条件: 0.2秒間の角度変化量が±10°以内`);

    // 主要な飛行フェーズの角度変化をログ出力
    const flightPhases = [
        { name: "発射台離脱時", time: Math.min(PHYSICAL_CONSTANTS.launchRailLength / Math.max(0.1, Math.sqrt(Math.pow(data[0].vx, 2) + Math.pow(data[0].vy, 2))), thrustEndTime) },
        { name: "推力終了時", time: thrustEndTime },
        { name: "最高点", time: keyPoints.maxHeight.time },
        { name: "パラシュート展開時", time: keyPoints.parachuteEjection?.time || MAX_TIME }
    ];

    flightPhases.forEach(phase => {
        const phaseIndex = Math.min(Math.floor(phase.time / dt), data.length - 1);
        if (phaseIndex >= 0 && phaseIndex < data.length) {
            const phaseData = data[phaseIndex];
            console.log(`${phase.name} (t=${phase.time.toFixed(2)}s): 角度=${phaseData.omegaDegrees.toFixed(2)}°, 角度偏差=${phaseData.angleDeviationDegrees.toFixed(2)}°`);
        }
    });

    return {
        data,
        maxHeight,
        maxSpeed,
        maxDistance,
        maxFinDeflection,
        keyPoints,
        angleStability: {
            maxAngleChangePerDt2,
            isAngleStableOK
        },
        projectedAreas,  // 投影面積データを追加
        volumes,         // 体積データを追加
        pressureCenter: centerOfPressure,  // 圧力中心データを追加
        aerodynamicCenter,  // 空力中心データを追加
        stabilityCenterOfPressure, // 静安定用圧力中心を追加
        staticMargins,  // 静安定マージンを追加
        calculations: {
            aerodynamicCenter: Math.round(aerodynamicCenter.aerodynamicCenter),
            pressureCenter: Math.round(centerOfPressure.centerOfPressure),
            stabilityCenterOfPressure: Math.round(stabilityCenterOfPressure.stabilityCenterOfPressure),
            standardStaticMargin: parseFloat(staticMargins.standardStaticMargin.toFixed(2)),
            stabilityStaticMargin: parseFloat(staticMargins.stabilityStaticMargin.toFixed(2)),
            finDivergenceSpeed: Math.round(calculateFinDivergenceSpeed(rocketParams)),
            finFlutterSpeed: Math.round(calculateFinFlutterSpeed(rocketParams))
        },
        angleStability: {
            maxAngleChangePerDt2,
            isAngleStableOK,
            isAbsoluteAngleOK,
            isStabilityOverallOK: isAngleStableOK && isAbsoluteAngleOK, // 総合的な判定
            maxAbsoluteAngle: Math.max(...data.map(d => Math.abs(d.absoluteAngleDegrees || 0))), // 最大絶対角度
          }
    };
};
