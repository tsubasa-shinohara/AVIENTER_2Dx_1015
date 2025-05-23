// メインのシミュレーターコンポーネント
import React, { useState, useCallback, useEffect, useMemo, useRef } from 'react';
import { useNavigate } from 'react-router-dom';
// 定数とデータ定義のインポート
import {
  SVG_CONFIG, MOTOR_THRUST_DATA, PARACHUTE_SIZES, FIN_MATERIALS,
  NOSE_SHAPES, WIND_PROFILES, PHYSICAL_CONSTANTS, ANGLE_RESPONSE_DT,
  ANGLE_STEPS_PER_UPDATE, ANIMATION_SPEED, SLIDER_DEBOUNCE_TIME,
  mmToM, gToKg, UI_CONFIG, ANALYSIS_VIEW_CONFIG
} from './RocketConstants';

// 物理計算関連のインポート
import {
  calculateProjectedArea, calculateVolume, calculateCenterOfPressure,
  calculateAerodynamicCenter, calculateStabilityCenterOfPressure,
  calculateStaticMargin, calculateFinDivergenceSpeed, calculateFinFlutterSpeed,
  formatFinDeflection, formatSpeedValue, calculateWindSpeedAtHeight,
  calculateFlightPath, ENHANCED_ATTITUDE_CONTROL, WIND_ANGLE_LIMITATION,
  PHYSICAL_ATTITUDE_CONTROL // 姿勢制御関連の定数をインポート
} from './RocketPhysics';

// SVG描画関連のインポート
import {
  getNosePath, getBodyPath, getLeftFinPath, getRightFinPath, getCenterFinsPath,
  getParachutePath, getParachuteStringPaths, getWindArrow, getWindProfileArrows,
  metersToSvgX, metersToSvgY, getActualRocketDimensions, getSafeRotationTransform,
  getAnalysisViewBox, getDesignViewBox, getRocketTransform, getSafeValue,
} from './RocketRendering';

// UI関連のコンポーネントのインポート
import {
  WindAngleLimitVisualizer, ResultsPopup, LastFlightResults,
  ParameterSlider, DesignTab, AnalysisTab, SimulationTab
} from './RocketUIComponents';

// 新しい着地予測関連のインポートを追加
import { predictLanding, calculateFlightPathWithLanding } from './RocketLandingPrediction';


// ロケットデザインとシミュレーションを統合したカスタムフック
export const usePreFlightRocketSim = () => {
  // 初期値を明示的に設定することで、undefinedやnullが発生しない
  const initialRocketState = {
    noseShape: "ogive",
    noseHeight: 57,
    bodyHeight: 255,
    bodyWidth: 31,
    finHeight: 58.5,
    finBaseWidth: 65,
    finTipWidth: 25,
    finThickness: 1.5,
    finSweepLength: 95,
    finMaterial: "light_veneer",
    finCount: 3,
    weight: 50,
    centerOfGravity: 150
  };

  // Design parameters
  const [noseShape, setNoseShape] = useState("ogive");
  const [noseHeight, setNoseHeight] = useState(57);   // 57mmに変更
  const [bodyHeight, setBodyHeight] = useState(255);   // 255mmに変更
  const [bodyWidth, setBodyWidth] = useState(31);
  const [finHeight, setFinHeight] = useState(58.5);      // 58.5mmに変更
  const [finBaseWidth, setFinBaseWidth] = useState(65); // 65mmに変更
  const [finTipWidth, setFinTipWidth] = useState(25);   // 25mmに変更
  const [finThickness, setFinThickness] = useState(1.5); // 1.5mmに変更
  const [finSweepLength, setFinSweepLength] = useState(95); // 95mmに変更
  const [finMaterial, setFinMaterial] = useState("light_veneer");
  // フィン枚数状態を追加
  const [finCount, setFinCount] = useState(3); // デフォルトは4枚

  // Analysis parameters - weight変数の宣言を初期化前の参照より前に移動
  const [weight, setWeight] = useState(50);
  const [centerOfGravity, setCenterOfGravity] = useState(150);
  const [selectedMotor, setSelectedMotor] = useState("A8-3");
  const [selectedParachute, setSelectedParachute] = useState("φ300");

  // Simulation parameters
  const [launchAngle, setLaunchAngle] = useState(0);
  const [windSpeed, setWindSpeed] = useState(0);
  const [windProfile, setWindProfile] = useState("uniform");
  const [showWindArrows, setShowWindArrows] = useState(true);

  // rocketSimの初期状態が完全に構築されてから計算や描画を行うための状態
  const [isInitialized, setIsInitialized] = useState(false);

  const [enhancedAttitudeControl, setEnhancedAttitudeControl] = useState(ENHANCED_ATTITUDE_CONTROL);
  const [windAngleLimitation, setWindAngleLimitation] = useState(WIND_ANGLE_LIMITATION);

  // 初期化完了を検出するuseEffect
  useEffect(() => {
    // すべての必要な値が初期化されたことを確認
    if (
      noseShape &&
      typeof noseHeight === 'number' &&
      typeof bodyHeight === 'number' &&
      typeof bodyWidth === 'number' &&
      typeof finHeight === 'number' &&
      typeof finBaseWidth === 'number' &&
      typeof finTipWidth === 'number' &&
      typeof finThickness === 'number' &&
      typeof finSweepLength === 'number' &&
      typeof weight === 'number' &&
      typeof centerOfGravity === 'number'
    ) {
      setIsInitialized(true);
    }
  }, [
    noseShape, noseHeight, bodyHeight, bodyWidth,
    finHeight, finBaseWidth, finTipWidth, finThickness, finSweepLength,
    weight, centerOfGravity
  ]);

  // Simulation state
  const [isLaunched, setIsLaunched] = useState(false);
  const [preFlightData, setPreFlightData] = useState([]);
  const [currentTime, setCurrentTime] = useState(0);
  const [animationId, setAnimationId] = useState(null);
  const [trajectoryScale, setTrajectoryScale] = useState(1.0);
  const [rocketScale, setRocketScale] = useState(0.1);
  const [currentHeight, setCurrentHeight] = useState(0);
  const [currentSpeed, setCurrentSpeed] = useState(0);
  const [currentDistance, setCurrentDistance] = useState(0);
  const [currentFinDeflection, setCurrentFinDeflection] = useState(0);
  const [currentMaxHeight, setCurrentMaxHeight] = useState(0);
  const [currentMaxSpeed, setCurrentMaxSpeed] = useState(0);
  const [currentMaxDistance, setCurrentMaxDistance] = useState(0);
  const [currentMaxFinDeflection, setCurrentMaxFinDeflection] = useState(0);
  const [completedFlights, setCompletedFlights] = useState([]);
  const [keyPoints, setKeyPoints] = useState({});

  // useRocketSimulator 内で、他の状態変数と一緒に追加
  const [prec_MaxHeight, setPrec_MaxHeight] = useState(0);
  const [isPreLaunched, setIsPreLaunched] = useState(false);

  // 状態変数の追加
  const [showResultsPopup, setShowResultsPopup] = useState(false);
  const [preFlightResults, setPreFlightResults] = useState(null);
  const [lastFlightResults, setLastFlightResults] = useState(null);

  // 新しく追加した状態変数
  const [projectedAreas, setProjectedAreas] = useState(null);
  const [volumes, setVolumes] = useState(null);
  const [pressureCenter, setPressureCenter] = useState(null);
  const [aerodynamicCenter, setAerodynamicCenter] = useState(null);
  const [stabilityCenterOfPressure, setStabilityCenterOfPressure] = useState(null);
  const [staticMargins, setStaticMargins] = useState(null);

  // 着地予測関連の状態変数を追加
  const [landing, setLanding] = useState(null);
  const [showLandingPrediction, setShowLandingPrediction] = useState(true);

  // 物理計算のための全パラメータをまとめる
  const preSimParams = useMemo(() => ({
    noseShape,
    noseHeight,
    bodyHeight,
    bodyWidth,
    finHeight,
    finBaseWidth,
    finTipWidth,
    finThickness,
    finSweepLength,
    finMaterial,
    weight,
    centerOfGravity,
    selectedMotor,
    selectedParachute,
    finCount // finCountパラメータを追加
  }), [
    noseShape, noseHeight, bodyHeight, bodyWidth,
    finHeight, finBaseWidth, finTipWidth, finThickness, finSweepLength,
    finMaterial, weight, centerOfGravity, selectedMotor, selectedParachute,
    finCount // 依存配列にもfinCountを追加
  ]);

  // Design display dimensions
  const design = useMemo(() => ({
    width: UI_CONFIG.designViewWidth,
    height: UI_CONFIG.designViewHeight,
    centerX: UI_CONFIG.designViewWidth / 2
  }), []);

  const analysis = useMemo(() => ({
    width: UI_CONFIG.analysisViewWidth,
    height: UI_CONFIG.analysisViewHeight,
    centerX: UI_CONFIG.analysisViewWidth / 2
  }), []);

  // 実際のロケット寸法を計算するためのラッパー
  const getRocketDimensions = useCallback((config) => {
    return getActualRocketDimensions(
      config, noseHeight, bodyHeight,
      finSweepLength, finTipWidth, finBaseWidth
    );
  }, [noseHeight, bodyHeight, finSweepLength, finTipWidth, finBaseWidth]);

  // 現在の位置情報を取得
  const getCurrentPosition = useCallback(() => {
    if (!isLaunched || !preFlightData || preFlightData.length === 0) {
      // 初期状態・未発射状態
      return {
        physicsX: 0,
        physicsY: 0,
        rotation: launchAngle, // 初期角度（発射前）
        isParachuteEjected: false,
        isParachuteActive: false,
        parachuteDeploymentProgress: 0,
        horizontalDistance: 0,
        finDeflection: 0,
        angleChangePerDt2: 0,
        effectiveWindSpeed: windSpeed
      };
    }

    const timeIndex = Math.min(Math.floor(currentTime / 0.02), preFlightData.length - 1);
    if (timeIndex < 0 || timeIndex >= preFlightData.length) {
      return {
        physicsX: 0,
        physicsY: 0,
        rotation: launchAngle,
        isParachuteEjected: false,
        isParachuteActive: false,
        parachuteDeploymentProgress: 0,
        horizontalDistance: 0,
        finDeflection: 0,
        angleChangePerDt2: 0,
        effectiveWindSpeed: windSpeed
      };
    }

    const currentData = preFlightData[timeIndex];
    if (!currentData) {
      return {
        physicsX: 0,
        physicsY: 0,
        rotation: launchAngle,
        isParachuteEjected: false,
        isParachuteActive: false,
        parachuteDeploymentProgress: 0,
        horizontalDistance: 0,
        finDeflection: 0,
        angleChangePerDt2: 0,
        effectiveWindSpeed: windSpeed
      };
    }












    // すべての数値にNaNチェックを追加
    return {
      physicsX: isNaN(currentData.physicsX) ? 0 : currentData.physicsX,
      physicsY: isNaN(currentData.physicsY) ? 0 : currentData.physicsY,
      rotation: isNaN(currentData.omega) ? launchAngle : (currentData.omega * 180 / Math.PI),
      isParachuteEjected: currentData.isParachuteEjected || false,
      isParachuteActive: currentData.isParachuteActive || false,
      parachuteDeploymentProgress: isNaN(currentData.parachuteDeploymentProgress) ? 0 :
        (currentData.parachuteDeploymentProgress || 0),
      torque: isNaN(currentData.torque) ? 0 : (currentData.torque || 0),
      angleChangePerDt2: isNaN(currentData.angleChangePerDt2) ? 0 : (currentData.angleChangePerDt2 || 0),
      horizontalDistance: isNaN(currentData.horizontalDistance) ? 0 :
        (currentData.horizontalDistance || Math.abs(isNaN(currentData.physicsX) ? 0 : currentData.physicsX) || 0),
      finDeflection: isNaN(currentData.finDeflection) ? 0 : (currentData.finDeflection || 0),
      effectiveWindSpeed: isNaN(currentData.effectiveWindSpeed) ? windSpeed : (currentData.effectiveWindSpeed || windSpeed)
    };
  }, [isLaunched, preFlightData, currentTime, launchAngle, windSpeed]);

  // モーターに応じた初期スケール値を取得する関数
  const getInitialScaleForMotor = (motorType) => {
    const baseScales = {
      '1/2A6-2': 16,  // 強化された値
      'A8-3': 12,     // 強化された値
      'B6-4': 8       // 強化された値
    };
    return baseScales[motorType] || 12; // デフォルト値も強化
  };

  // モーターのパワー係数を取得する関数
  const getMotorPowerFactor = (motorType) => {
    const powerFactors = {
      '1/2A6-2': 0.6, // 調整された値
      'A8-3': 0.5,    // 調整された値
      'B6-4': 0.4     // 調整された値
    };
    return powerFactors[motorType] || 0.5; // デフォルト値も調整
  };

  // preCalculateFlightPath関数の修正
  const preCalculateFlightPath = useCallback(() => {
    console.log('事前飛行経路を再計算します');

    try {
      // 現在進行中のアニメーションがあれば中止する
      if (isLaunched && animationId) {
        cancelAnimationFrame(animationId);
        setAnimationId(null);
        setIsLaunched(false);
      }

      // 風速プロファイルを引数として渡す
      const preFlight = calculateFlightPath(
        preSimParams,
        launchAngle,
        windSpeed,
        windProfile,
        {
          ...SVG_CONFIG,
          enhancedAttitudeControl,
          windAngleLimitation
        }
      );







      if (preFlight && preFlight.prec_MaxHeight > 0) {
        // 最大高度を保存 - この行が重要
        console.log(`最大高度を更新: ${preFlight.prec_MaxHeight}m`);
        // ここで直接更新せず、値を保持
        const newMaxHeight = preFlight.prec_MaxHeight;






        // 改良：より高いベース高さを設定
        const baseHeights = {
          '1/2A6-2': 100,
          'A8-3': 150,
          'B6-4': 200
        };

        const expectedBaseHeight = baseHeights[preSimParams.selectedMotor] || 150;
        const targetHeight = Math.max(preFlight.maxHeight * 1.3, expectedBaseHeight);

        const minHorizontalDistance = expectedBaseHeight * 0.9;
        const maxDistance = Math.max(preFlight.maxDistance || 0, minHorizontalDistance);

        // スケール計算
        const verticalScale = availableHeight / targetHeight;
        const horizontalScale = (SVG_CONFIG.width * 0.9) / (maxDistance * 2 || 1);

        const motorPowerFactor = {
          '1/2A6-2': 0.45,
          'A8-3': 0.35,
          'B6-4': 0.25
        };

        const powerFactor = motorPowerFactor[preSimParams.selectedMotor] || 0.35;

        // 最小/最大スケール値の調整
        const minScale = 10;
        const maxScale = 24;

        const baseRocketScale = 0.03;

        const rawScale = Math.min(verticalScale, horizontalScale) * powerFactor;
        // 最終スケールを調整 - 必ず最小スケールを適用
        const finalScale = Math.max(minScale, Math.min(maxScale, rawScale));



        // 状態更新の部分を分離 - 結果を返すよう変更
        const results = {
          prec_MaxHeight: newMaxHeight,
          maxDistance: preFlight.maxDistance || 0,
          finalScale: finalScale, // 計算した最終スケール
          rocketScale: baseRocketScale * powerFactor
        };



        // 閾値による更新制限
        const diff = Math.abs(newMaxHeight - prec_MaxHeight);
        const threshold = 0.5; // 0.5m以上の差がある場合のみ更新

        if (diff > threshold || prec_MaxHeight === 0) {
          // 重要: 遅延して更新することで無限ループを防止
          setTimeout(() => {
            setPrec_MaxHeight(newMaxHeight);
          }, 0);
        }

        // スケールの更新は直接行う（これは毎回更新して問題ない）
        setTrajectoryScale(finalScale);
        setRocketScale(baseRocketScale * powerFactor);

        if (!isPreLaunched) {
          setIsPreLaunched(true);
        }


        // 非同期で状態更新を行う
        setTimeout(updateStates, 0);

        console.log(`事前計算完了: 高度=${newMaxHeight.toFixed(1)}m, スケール=${finalScale.toFixed(2)}`);
        return results;
      }
    } catch (error) {
      console.error('事前飛行経路計算エラー:', error);
    }










    // 失敗した場合はデフォルト値を設定
    const defaultScale = getInitialScaleForMotor(selectedMotor);

    // 非同期で状態更新を行う
    setTimeout(() => {
      setTrajectoryScale(defaultScale);
      setRocketScale(0.03 * getMotorPowerFactor(selectedMotor));

      if (!isPreLaunched) {
        setIsPreLaunched(true);
      }
    }, 0);

    return false;
  }, [
    preSimParams, launchAngle, windSpeed, windProfile,
    enhancedAttitudeControl, windAngleLimitation,
    isLaunched, animationId, selectedMotor
    // prec_MaxHeightとisPreLaunchedを依存配列から除外
  ]);


  // この関数を usePreFlightRocketSim フック内に追加
  useEffect(() => {
    if (!isInitialized) return;

    // パラメータが変更されたら自動的に計算を実行
    console.log('パラメータが変更されました: prec_MaxHeightの先行計算を実行します');

    try {
      // 非同期で計算を実行して UI をブロックしない
      const timer = setTimeout(() => {
        const preFlight = calculateFlightPath(
          preSimParams,
          launchAngle,
          windSpeed,
          windProfile,
          {
            ...SVG_CONFIG,
            enhancedAttitudeControl,
            windAngleLimitation
          }
        );

        if (preFlight && preFlight.prec_MaxHeight > 0) {
          const newMaxHeight = preFlight.prec_MaxHeight;
          // 変更が閾値を超える場合のみ更新
          const diff = Math.abs(newMaxHeight - prec_MaxHeight);
          const threshold = 0.5;

          if (diff > threshold || prec_MaxHeight === 0) {
            console.log(`先行計算: prec_MaxHeightを更新: ${prec_MaxHeight} -> ${newMaxHeight}m`);
            setPrec_MaxHeight(newMaxHeight);
          } else {
            console.log(`prec_MaxHeightの変更が小さいため更新をスキップ: ${diff.toFixed(2)}m`);
          }
        }
      }, 100); // UI更新を優先するため少し遅延させる

      return () => clearTimeout(timer);
    } catch (error) {
      console.warn("prec_MaxHeight先行計算に失敗しました", error);
    }
  }, [
    isInitialized,
    preSimParams,
    launchAngle,
    windSpeed,
    windProfile,
    enhancedAttitudeControl,
    windAngleLimitation
    // prec_MaxHeightは依存配列から除外して無限ループを防止
  ]);













  // リセット関数を強化
  const handleReset = useCallback(() => {
    console.log('リセット処理開始');

    // アニメーションをキャンセル
    if (animationId) {
      cancelAnimationFrame(animationId);
      setAnimationId(null);
    }

    // 現在の結果を前回の結果として保存
    if (preFlightResults) {
      setLastFlightResults(preFlightResults);
      setPreFlightResults(null);
    }

    // ポップアップを閉じる
    setShowResultsPopup(false);

    // すべての状態をリセット
    setIsLaunched(false);
    setPreFlightData([]);
    setCurrentTime(0);
    setCurrentHeight(0);
    setCurrentSpeed(0);
    setCurrentDistance(0);
    setCurrentFinDeflection(0);
    setCurrentMaxHeight(0);
    setCurrentMaxSpeed(0);
    setCurrentMaxDistance(0);
    setCurrentMaxFinDeflection(0);
    setLanding(null); // 着地予測をリセット

    // 過去の飛行軌跡をクリア
    setCompletedFlights([]);

    // モーターに応じた適切なスケールを設定
    const stableScale = getInitialScaleForMotor(selectedMotor);
    setTrajectoryScale(stableScale);

    // ロケットスケールも設定
    const rocketScaleFactor = 0.03 * getMotorPowerFactor(selectedMotor);
    setRocketScale(rocketScaleFactor);

    console.log('シミュレーションが完全にリセットされました。新しいスケール: ', stableScale);
  }, [animationId, preFlightResults, selectedMotor]);

  const handleLaunch = useCallback(() => {
    if (isLaunched) return;

    try {
      // 風速プロファイルを引数として渡す
      const preFlight = calculateFlightPathWithLanding(
        calculateFlightPath,
        preSimParams,
        launchAngle,
        windSpeed,
        windProfile,
        {
          ...SVG_CONFIG,
          enhancedAttitudeControl,
          windAngleLimitation
        }
      );

      if (!preFlight?.data?.length) {
        console.error('フライトデータが空です');
        return;
      }

      // エラーチェックを追加
      if (preFlight.error?.hasError) {
        console.error('シミュレーションでエラーが発生しました:', flight.error.message);

        // エラーポップアップを表示（エラー用の結果オブジェクトを作成）
        const errorResults = {
          isError: true,
          errorType: preFlight.error.type,
          errorMessage: preFlight.error.message,
          errorTime: preFlight.error.time,
          velocity: preFlight.error.velocity,
          finDivergenceSpeed: preFlight.error.finDivergenceSpeed,
          finFlutterSpeed: preFlight.error.finFlutterSpeed,
          maxDeflectionPercent: preFlight.error.maxDeflectionPercent,
          launchAngle,
          windSpeed,
          windProfile
        };

        setPreFlightData(errorResults);
        setShowResultsPopup(true);
        return; // 早期リターンでこれ以上の処理を行わない
      }

      // 事前計算に最大高度を保存 - これが重要2
      if (preFlight && preFlight.prec_MaxHeight > 0) {
        setPrec_MaxHeight(preFlight.prec_MaxHeight);
      }

      // 最大高度を保存 - これが重要
      //if (flight && flight.maxHeight > 0) {
      //  setMaxHeight(flight.maxHeight);
      //}

      // 初期データを取得（最初のフレーム用）
      const initialData = preFlight.data[0];
      console.log('初期フライトデータ：', initialData);
      console.log(`最高到達高度: ${preFlight.prec_MaxHeight.toFixed(2)}m, 最高速度: ${flight.maxSpeed.toFixed(2)}m/s, 最大水平距離: ${flight.maxDistance.toFixed(2)}m`);
      console.log(`最大フィンたわみ量: ${preFlight.maxFinDeflection.toFixed(4)}mm`);
      console.log(`姿勢安定性: 最大角度変化量=${preFlight.angleStability.maxAngleChangePerDt2.toFixed(2)}°`);
      // 絶対角度の判定結果も出力
      console.log(`絶対角度安定性: 最大絶対角度=${preFlight.angleStability.maxAbsoluteAngle?.toFixed(2) || 0}°, 判定=${flight.angleStability.isAbsoluteAngleOK ? 'OK' : 'NG'}`);

      // 着地予測情報をログ出力
      if (flight.landing) {
        console.log(`着地予測: 距離=${preFlight.landing.landingDistance.toFixed(2)}m, 時間=${flight.landing.timeToLanding.toFixed(2)}秒`);
      }

      // 着地予測情報を状態に保存
      setLanding(preFlight.landing);

      // キーポイントを保存
      setKeyPoints(preFlight.keyPoints);

      setPreFlightData(preFlight.data);
      setIsLaunched(true);
      setCurrentTime(0);

      // 初期値を明示的に設定
      setCurrentHeight(initialData.height || 0);
      setCurrentSpeed(initialData.vy || 0);
      setCurrentDistance(initialData.horizontalDistance || 0);
      setCurrentFinDeflection(initialData.finDeflection || 0);
      setCurrentMaxHeight(initialData.height || 0);
      setCurrentMaxSpeed(Math.abs(initialData.speedMagnitude) || 0);
      setCurrentMaxDistance(0); // 最大水平距離の初期値
      setCurrentMaxFinDeflection(0); // 最大フィンたわみ量の初期値

      // アニメーション開始時間を記録
      const startTime = performance.now();
      let firstFrame = true;

      const animate = (timestamp) => {
        // 初回フレームの特別処理
        if (firstFrame) {
          firstFrame = false;
          requestAnimationFrame(animate);
          return;
        }

        try {
          // アニメーション開始からの累積時間を計算（ANIMATION_SPEEDで調整）
          const elapsed = Math.max(0, (timestamp - startTime) / 1000 * ANIMATION_SPEED);

          // シミュレーションデータのインデックスを計算 - Math.maxを追加して負のインデックスを防止
          const timeIndex = Math.max(0, Math.min(Math.floor(elapsed / 0.02), preFlight.data.length - 1));

          // 追加のデバッグログ - 100フレームごとに状態を記録
          if (timeIndex % 100 === 0) {
            console.log(`Animation frame: time=${elapsed.toFixed(2)}s, index=${timeIndex}, total=${flight.data.length}`);
          }

          // データの安全性チェック
          if (timeIndex >= 0 && timeIndex < preFlight.data.length) {
            const currentData = preFlight.data[timeIndex];

            if (currentData && typeof currentData === 'object') {
              // NaNチェックを追加
              setCurrentTime(elapsed);
              setCurrentHeight(isNaN(currentData.height) ? 0 : currentData.height || 0);
              setCurrentSpeed(isNaN(currentData.vy) ? 0 : currentData.vy || 0);
              setCurrentDistance(isNaN(currentData.horizontalDistance) ? 0 :
                (currentData.horizontalDistance || Math.abs(isNaN(currentData.physicsX) ? 0 : currentData.physicsX) || 0));

              // フィンたわみ量のNaNチェック
              const finDeflection = isNaN(currentData.finDeflection) ? 0 : currentData.finDeflection || 0;
              setCurrentFinDeflection(finDeflection);

              // 最大値の安全な更新
              setCurrentMaxHeight(prev => Math.max(prev, isNaN(currentData.height) ? 0 : currentData.height || 0));
              setCurrentMaxSpeed(prev => Math.max(prev, isNaN(currentData.speedMagnitude) ? 0 :
                Math.abs(currentData.speedMagnitude) || 0));
              setCurrentMaxDistance(prev => Math.max(prev, isNaN(currentData.physicsX) ? 0 :
                Math.abs(currentData.physicsX) || 0));

              // 最大フィンたわみ量の更新
              setCurrentMaxFinDeflection(prev => Math.max(prev, finDeflection));

              // アニメーションをデータの最後まで続けるようにする
              if (timeIndex < preFlight.data.length - 1) {
                const id = requestAnimationFrame(animate);
                setAnimationId(id);
              } else {
                // アニメーション終了 - 最終フレームに到達した時だけ結果を表示
                console.log('アニメーション完了: 最終インデックス到達');

                // シミュレーションデータから最大値を直接計算（状態変数に依存せず確実に取得）
                const prec_MaxHeight = Math.max(...preFlight.data.map(d => isNaN(d.height) ? 0 : d.height || 0));
                const maxSpeed = Math.max(...preFlight.data.map(d => isNaN(d.speedMagnitude) ? 0 :
                  Math.abs(d.speedMagnitude) || 0));
                const maxDistance = Math.max(...preFlight.data.map(d => isNaN(d.physicsX) ? 0 :
                  Math.abs(d.physicsX) || 0));
                const maxFinDeflection = Math.max(...preFlight.data.map(d => isNaN(d.finDeflection) ? 0 :
                  d.finDeflection || 0));

                // 飛行結果の評価（直接計算した最大値を使用）
                const isDivergenceOK = maxSpeed < preFlight.calculations.finDivergenceSpeed;
                const isFlutterOK = maxSpeed < preFlight.calculations.finFlutterSpeed;
                const maxDeflectionPercent = (maxFinDeflection / finHeight) * 100;
                const isDeflectionOK = maxDeflectionPercent <= 3;

                // 姿勢安定性の判定結果 - シミュレーション結果から直接取得
                const isAngleStableOK = preFlight.angleStability.isAngleStableOK;
                const maxAngleChangePerDt2 = preFlight.angleStability.maxAngleChangePerDt2;
                // 絶対角度判定を追加 
                const isAbsoluteAngleOK = preFlight.angleStability.isAbsoluteAngleOK;
                const maxAbsoluteAngle = preFlight.angleStability.maxAbsoluteAngle;

                // 総合判定は全てのチェックをパスする必要がある
                // 絶対角度判定も含める
                const isOverallOK = isDivergenceOK && isFlutterOK && isDeflectionOK && isAngleStableOK && isAbsoluteAngleOK;

                // 結果オブジェクトの作成
                const results = {
                  maxSpeed: maxSpeed,
                  maxHeight: maxHeight,
                  maxDistance: maxDistance,
                  maxFinDeflection: maxFinDeflection,
                  finHeight: finHeight,
                  maxDeflectionPercent: maxDeflectionPercent,
                  divergenceSpeed: formatSpeedValue(preFlight.calculations.finDivergenceSpeed),
                  flutterSpeed: formatSpeedValue(preFlight.calculations.finFlutterSpeed),
                  isDivergenceOK,
                  isFlutterOK,
                  isDeflectionOK,
                  isAngleStableOK,
                  isAbsoluteAngleOK, // 絶対角度判定を追加
                  maxAngleChangePerDt2,
                  maxAbsoluteAngle, // 最大絶対角度を追加
                  isOverallOK,
                  launchAngle,
                  windSpeed,
                  windProfile,
                  landing: preFlight.landing // 着地予測情報を追加
                };

                // 状態更新を一度に行う
                setPreFlightResults(results);
                setShowResultsPopup(true);

                // UI上の最大値表示も更新
                setCurrentMaxHeight(maxHeight);
                setCurrentMaxSpeed(maxSpeed);
                setCurrentMaxDistance(maxDistance);
                setCurrentMaxFinDeflection(maxFinDeflection);

                const completedFlightData = {
                  data: [...preFlight.data],
                  launchAngle,
                  windSpeed,
                  windProfile,
                  Prec_MaxHeight: prec_MaxHeight,
                  maxSpeed: maxSpeed,
                  maxDistance: maxDistance,
                  maxFinDeflection: maxFinDeflection,
                  scale: trajectoryScale,
                  results: results // 結果も保存
                };

                setCompletedFlights(prevFlights => [...prevFlights, completedFlightData]);
                setIsLaunched(false);
              }
            }
          }
        } catch (error) {
          // エラーが発生した場合は記録して次のフレームを試行
          console.error('Animation frame error:', error);

          // 重大なエラーでなければ次のフレームを継続
          if (timeIndex < preFlight.data.length - 1) {
            const id = requestAnimationFrame(animate);
            setAnimationId(id);
          } else {
            // エラー発生時も最終フレームに達していれば終了処理
            handleReset();
          }
        }
      };

      // アニメーション開始
      requestAnimationFrame(animate);
    } catch (error) {
      console.error('Launch error:', error);
      handleReset();
    }
  }, [isLaunched, launchAngle, windSpeed, windProfile, preSimParams, handleReset, finHeight, trajectoryScale]);


  // 本フックから公開する関数とパラメータ
  return {


    // シミュレーション状態
    isLaunched, setIsLaunched,
    preFlightData,
    currentTime,
    currentHeight, currentSpeed, currentDistance, currentFinDeflection,
    currentMaxHeight, currentMaxSpeed, currentMaxDistance, currentMaxFinDeflection,
    completedFlights, keyPoints,

    // Setterの一覧
    noseShape, setNoseShape,
    noseHeight, setNoseHeight,
    bodyHeight, setBodyHeight,
    bodyWidth, setBodyWidth,
    finHeight, setFinHeight,
    finBaseWidth, setFinBaseWidth,
    finTipWidth, setFinTipWidth,
    finThickness, setFinThickness,
    finSweepLength, setFinSweepLength,
    finMaterial, setFinMaterial,
    finCount, setFinCount,
    weight, setWeight,
    centerOfGravity, setCenterOfGravity,
    selectedMotor, setSelectedMotor,
    selectedParachute, setSelectedParachute,

    preCalculateFlightPath, // RocketSimulator.jsxの中のIntegretedRocketSimulator関数内で使用
    prec_MaxHeight, // prec_MaxHeightも外部に公開

    // 表示設定
    design, analysis,
    trajectoryScale, rocketScale,

    // 新しい計算結果
    projectedAreas, volumes, pressureCenter, aerodynamicCenter, stabilityCenterOfPressure, staticMargins,


    // 初期化状態
    isInitialized,

    // ユーティリティ関数
    getActualRocketDimensions: getRocketDimensions,
    getCurrentPosition,
    calculateWindSpeedAtHeight,
    formatFinDeflection,
    getSafeValue,

    // 姿勢制御設定
    enhancedAttitudeControl,
    setEnhancedAttitudeControl,
    windAngleLimitation,
    setWindAngleLimitation,

    // 着地予測関連
    landing,
    showLandingPrediction,
    setShowLandingPrediction,

    // 操作関数
    handleLaunch,
    handleReset,


  };
};