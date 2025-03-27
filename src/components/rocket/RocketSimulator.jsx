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

// 開発モード設定 - 本番環境ではfalseに設定する
const ENABLE_DEV_MODE = false; // ここを true/false で切り替える

// ロケットデザインとシミュレーションを統合したカスタムフック
const useRocketSimulator = () => {
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
  const [finCount, setFinCount] = useState(4); // デフォルトは4枚

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
  const [flightData, setFlightData] = useState([]);
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

  // 状態変数の追加
  const [showResultsPopup, setShowResultsPopup] = useState(false);
  const [flightResults, setFlightResults] = useState(null);
  const [lastFlightResults, setLastFlightResults] = useState(null);

  // 新しく追加した状態変数
  const [projectedAreas, setProjectedAreas] = useState(null);
  const [volumes, setVolumes] = useState(null);
  const [pressureCenter, setPressureCenter] = useState(null);
  const [aerodynamicCenter, setAerodynamicCenter] = useState(null);
  const [stabilityCenterOfPressure, setStabilityCenterOfPressure] = useState(null);
  const [staticMargins, setStaticMargins] = useState(null);

  // 物理計算のための全パラメータをまとめる
  const simulationParams = useMemo(() => ({
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

  // useRocketSimulator内で、noseShapeの状態変更を正しく伝播するように修正
  const setNoseShapeAndUpdate = (shape) => {
    setNoseShape(shape);
    // その他の更新は依存関係に基づいてuseEffectで自動的に行われる
  };

  // スライダーの動きをスムーズにするためのdebounce用ref
  const sliderTimeoutRef = useRef(null);

  // スライダーのリファレンスを保持するための変数
  const noseHeightInputRef = useRef(null);
  const bodyHeightInputRef = useRef(null);
  const bodyWidthInputRef = useRef(null);
  const finHeightInputRef = useRef(null);
  const finBaseWidthInputRef = useRef(null);
  const finTipWidthInputRef = useRef(null);
  const finThicknessInputRef = useRef(null);
  const finSweepLengthInputRef = useRef(null);
  const weightInputRef = useRef(null);
  const centerOfGravityInputRef = useRef(null);
  const launchAngleInputRef = useRef(null);
  const windSpeedInputRef = useRef(null);

  // スムーズなスライダー操作のためのdebounce関数
  const debounce = (func) => {
    // デバウンスを実質的に無効化して直接値を更新
    return (...args) => {
      func(...args);
    };
  };

  // スライダー値の更新処理をよりダイレクトにするハンドラーを追加
  const handleSliderChange = useCallback((ref, setter) => {
    // マウスイベントの処理関数
    const updateValueFromEvent = (event) => {
      if (!ref.current) return;

      const rect = ref.current.getBoundingClientRect();
      const width = rect.width;
      const left = rect.left;

      // マウス位置をスライダー内の相対位置(0～1)に変換
      let relativeX = (event.clientX - left) / width;
      // 範囲外の場合はクランプ(0～1に制限)
      relativeX = Math.max(0, Math.min(1, relativeX));

      // スライダーの最小値と最大値
      const min = Number(ref.current.min);
      const max = Number(ref.current.max);
      const step = Number(ref.current.step) || 1;

      // 相対位置から実際の値を計算
      let newValue = min + relativeX * (max - min);

      // 設定されたstepに基づいて値を丸める
      if (step !== 0) {
        newValue = Math.round(newValue / step) * step;
      }

      // 念のため、最終値を最小値と最大値の範囲内に収める
      newValue = Math.max(min, Math.min(max, newValue));

      // 値を更新 - 常にNumberとして扱う
      setter(Number(newValue));
    };

    return {
      // 値変更イベント(標準のonChange) - 明示的にNumberに変換
      onChange: (e) => setter(Number(e.target.value)),

      // トラックのクリックイベント(スライダー上の任意の位置)
      onClick: updateValueFromEvent,

      // スライダーのドラッグ開始
      onMouseDown: (startEvent) => {
        // ドキュメント全体でのマウス移動を監視
        const handleMouseMove = (moveEvent) => {
          moveEvent.preventDefault();
          updateValueFromEvent(moveEvent);
        };

        // マウスボタンを離したときの処理
        const handleMouseUp = () => {
          document.removeEventListener('mousemove', handleMouseMove);
          document.removeEventListener('mouseup', handleMouseUp);
        };

        // 最初のクリック位置で値を更新
        updateValueFromEvent(startEvent);

        // ドキュメント全体にイベントリスナーを追加
        document.addEventListener('mousemove', handleMouseMove);
        document.addEventListener('mouseup', handleMouseUp);
      },

      // タッチデバイス対応
      onTouchStart: (startEvent) => {
        if (!startEvent.touches[0]) return;

        const handleTouchMove = (moveEvent) => {
          if (!moveEvent.touches[0]) return;
          moveEvent.preventDefault();

          const touch = moveEvent.touches[0];
          const mockEvent = { clientX: touch.clientX, clientY: touch.clientY };
          updateValueFromEvent(mockEvent);
        };

        const handleTouchEnd = () => {
          document.removeEventListener('touchmove', handleTouchMove);
          document.removeEventListener('touchend', handleTouchEnd);
        };

        const touch = startEvent.touches[0];
        const mockEvent = { clientX: touch.clientX, clientY: touch.clientY };
        updateValueFromEvent(mockEvent);

        document.addEventListener('touchmove', handleTouchMove, { passive: false });
        document.addEventListener('touchend', handleTouchEnd);
      }
    };
  }, []);

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

  // メートル単位からSVG座標への変換関数の修正版
  const convertMetersToSvgX = useCallback((meters) => {
    return metersToSvgX(meters, trajectoryScale);
  }, [trajectoryScale]);

  const convertMetersToSvgY = useCallback((meters) => {
    return metersToSvgY(meters, trajectoryScale);
  }, [trajectoryScale]);

  // 実際のロケット寸法を計算するためのラッパー
  const getRocketDimensions = useCallback((config) => {
    return getActualRocketDimensions(
      config, noseHeight, bodyHeight,
      finSweepLength, finTipWidth, finBaseWidth
    );
  }, [noseHeight, bodyHeight, finSweepLength, finTipWidth, finBaseWidth]);

  // 視覚化のための風速高度分布矢印ラッパー
  const getWindArrows = useCallback((baseWindSpeed, profile) => {
    return getWindProfileArrows(
      baseWindSpeed, profile, showWindArrows,
      convertMetersToSvgY, calculateWindSpeedAtHeight
    );
  }, [showWindArrows, convertMetersToSvgY]);

  // パラシュート描画関数のラッパー
  const getParachute = useCallback((x, y, isOpen, deploymentProgress, rotation) => {
    const rocketLength = mmToM(noseHeight + bodyHeight);
    return getParachutePath(
      x, y, isOpen, deploymentProgress, rotation,
      rocketLength, selectedParachute, trajectoryScale
    );
  }, [noseHeight, bodyHeight, selectedParachute, trajectoryScale]);

  const getParachuteStrings = useCallback((x, y, isOpen, deploymentProgress, rotation) => {
    const rocketLength = mmToM(noseHeight + bodyHeight);
    return getParachuteStringPaths(
      x, y, isOpen, deploymentProgress, rotation,
      rocketLength, selectedParachute, trajectoryScale
    );
  }, [noseHeight, bodyHeight, selectedParachute, trajectoryScale]);

  // viewBox計算関数のラッパー
  const computeDesignViewBox = useCallback(() => {
    return getDesignViewBox(
      design, noseHeight, bodyHeight,
      finHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth
    );
  }, [design, noseHeight, bodyHeight, finHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth]);

  const computeAnalysisViewBox = useCallback(() => {
    return getAnalysisViewBox(
      analysis, noseHeight, bodyHeight,
      finHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth
    );
  }, [analysis, noseHeight, bodyHeight, finHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth]);

  // 初期化後の計算値の更新
  useEffect(() => {
    if (!isInitialized) return;

    // 投影面積、体積、圧力中心位置、空力中心位置を計算
    const areas = calculateProjectedArea(simulationParams);
    const volumeData = calculateVolume(simulationParams);
    const cpData = calculateCenterOfPressure(simulationParams);
    const acData = calculateAerodynamicCenter(simulationParams);
    const stabilityCpData = calculateStabilityCenterOfPressure(simulationParams);
    const margins = calculateStaticMargin(simulationParams);

    // 状態を更新
    setProjectedAreas(areas);
    setVolumes(volumeData);
    setPressureCenter(cpData);
    setAerodynamicCenter(acData);
    setStabilityCenterOfPressure(stabilityCpData);
    setStaticMargins(margins);

  }, [isInitialized, simulationParams]);

  // 動的に計算結果を更新する
  useEffect(() => {
    // 投影面積、体積、圧力中心位置、空力中心位置を計算
    const areas = calculateProjectedArea(simulationParams);
    const volumeData = calculateVolume(simulationParams);
    const cpData = calculateCenterOfPressure(simulationParams);
    const acData = calculateAerodynamicCenter(simulationParams);
    const stabilityCpData = calculateStabilityCenterOfPressure(simulationParams);
    const margins = calculateStaticMargin(simulationParams);

    // 状態を更新
    setProjectedAreas(areas);
    setVolumes(volumeData);
    setPressureCenter(cpData);
    setAerodynamicCenter(acData);
    setStabilityCenterOfPressure(stabilityCpData);
    setStaticMargins(margins);

  }, [simulationParams]);

  // 計算結果のキャッシュ
  const calculations = useMemo(() => {
    // 機体本体の高さ
    const actualRocketHeight = noseHeight + bodyHeight;
    // フィンの後端がボディ後端よりも出る場合の計算
    const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);
    // 全長は機体長 + フィン後端部分
    const totalHeight = actualRocketHeight + finExtension;

    // 圧力中心位置と空力中心位置（新しい計算結果を使用）
    const cp = pressureCenter?.centerOfPressure || (totalHeight * 0.7);
    const ac = aerodynamicCenter?.aerodynamicCenter || (totalHeight * 0.65);
    const stabilityCp = stabilityCenterOfPressure?.stabilityCenterOfPressure || (totalHeight * 0.6);

    // 姿勢安定性マージン - 重心位置との距離
    const standardMargin = staticMargins?.standardStaticMargin || ((cp - centerOfGravity) / bodyWidth);
    const stabilityMargin = staticMargins?.stabilityStaticMargin || ((stabilityCp - centerOfGravity) / bodyWidth);

    // 新しい計算関数を使用してフィン限界速度を計算
    const rocketParams = {
      noseHeight, bodyHeight, bodyWidth, finHeight, finBaseWidth, finTipWidth,
      finThickness, finSweepLength, finMaterial, centerOfGravity, weight
    };

    const finDivergenceSpeed = calculateFinDivergenceSpeed(rocketParams);
    const finFlutterSpeed = calculateFinFlutterSpeed(rocketParams);

    return {
      totalHeight: totalHeight,
      actualRocketHeight: actualRocketHeight,
      finExtension: finExtension,
      aerodynamicCenter: Math.round(ac),
      pressureCenter: Math.round(cp),
      stabilityCenterOfPressure: Math.round(stabilityCp),
      standardStaticMargin: standardMargin.toFixed(2),
      stabilityStaticMargin: stabilityMargin.toFixed(2),
      finDivergenceSpeed: Math.round(finDivergenceSpeed),
      finFlutterSpeed: Math.round(finFlutterSpeed),
      // フォーマット済みの値を追加
      finDivergenceSpeedDisplay: formatSpeedValue(finDivergenceSpeed),
      finFlutterSpeedDisplay: formatSpeedValue(finFlutterSpeed)
    };
  }, [noseHeight, bodyHeight, finSweepLength, finTipWidth, finBaseWidth, bodyWidth, centerOfGravity,
    pressureCenter, aerodynamicCenter, stabilityCenterOfPressure, staticMargins,
    finHeight, finThickness, finMaterial, weight]);

  // 現在の飛行フェーズを取得する関数
  const getCurrentFlightPhase = useCallback(() => {
    if (!isLaunched || !flightData || flightData.length === 0) {
      return "未発射";
    }

    // プロパティの安全性を考慮して独自に計算
    const timeIndex = Math.min(Math.floor(currentTime / 0.02), flightData.length - 1);
    if (timeIndex < 0 || timeIndex >= flightData.length) {
      return "未発射";
    }

    const currentData = flightData[timeIndex];
    if (!currentData) {
      return "未発射";
    }

    const position = {
      isParachuteActive: currentData.isParachuteActive,
      isParachuteEjected: currentData.isParachuteEjected
    };

    if (position.isParachuteActive) {
      return "パラシュート降下";
    } else if (position.isParachuteEjected) {
      return "パラシュート展開中";
    }

    // 発射台からの距離を計算
    const distanceFromStart = Math.sqrt(
      currentData.physicsX * currentData.physicsX +
      currentData.physicsY * currentData.physicsY
    );

    const onLaunchRail = distanceFromStart < PHYSICAL_CONSTANTS.launchRailLength;

    if (onLaunchRail) {
      return "発射台上";
    }

    // エンジン推力が残っているか確認
    const thrustEndTime = MOTOR_THRUST_DATA[selectedMotor].length * 0.02;
    if (currentTime <= thrustEndTime) {
      return "推力飛行";
    }

    return "慣性飛行";
  }, [isLaunched, flightData, currentTime, selectedMotor]);

  // 現在の位置情報を取得
  const getCurrentPosition = useCallback(() => {
    if (!isLaunched || !flightData || flightData.length === 0) {
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

    const timeIndex = Math.min(Math.floor(currentTime / 0.02), flightData.length - 1);
    if (timeIndex < 0 || timeIndex >= flightData.length) {
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

    const currentData = flightData[timeIndex];
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
  }, [isLaunched, flightData, currentTime, launchAngle, windSpeed]);

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

  // リセット関数を強化
  const handleReset = useCallback(() => {
    console.log('リセット処理開始');

    // アニメーションをキャンセル
    if (animationId) {
      cancelAnimationFrame(animationId);
      setAnimationId(null);
    }

    // 現在の結果を前回の結果として保存
    if (flightResults) {
      setLastFlightResults(flightResults);
      setFlightResults(null);
    }

    // ポップアップを閉じる
    setShowResultsPopup(false);

    // すべての状態をリセット
    setIsLaunched(false);
    setFlightData([]);
    setCurrentTime(0);
    setCurrentHeight(0);
    setCurrentSpeed(0);
    setCurrentDistance(0);
    setCurrentFinDeflection(0);
    setCurrentMaxHeight(0);
    setCurrentMaxSpeed(0);
    setCurrentMaxDistance(0);
    setCurrentMaxFinDeflection(0);

    // 過去の飛行軌跡をクリア
    setCompletedFlights([]);

    // モーターに応じた適切なスケールを設定
    const stableScale = getInitialScaleForMotor(selectedMotor);
    setTrajectoryScale(stableScale);

    // ロケットスケールも設定
    const rocketScaleFactor = 0.03 * getMotorPowerFactor(selectedMotor);
    setRocketScale(rocketScaleFactor);

    console.log('シミュレーションが完全にリセットされました。新しいスケール: ', stableScale);
  }, [animationId, flightResults, selectedMotor]);

  // ポップアップを閉じる関数
  const handleCloseResultsPopup = useCallback(() => {
    setShowResultsPopup(false);
    // 現在の結果を前回の結果として保存
    if (flightResults) {
      setLastFlightResults(flightResults);
    }
  }, [flightResults]);

  const handleLaunch = useCallback(() => {
    if (isLaunched) return;

    try {
      // 風速プロファイルを引数として渡す
      const flight = calculateFlightPath(
        simulationParams,
        launchAngle,
        windSpeed,
        windProfile, // 風速プロファイルを追加
        {
          ...SVG_CONFIG,
          enhancedAttitudeControl, // 拡張姿勢制御フラグを渡す
          windAngleLimitation     // 風向きによる角度制限フラグを渡す
        }
      );

      if (!flight?.data?.length) {
        console.error('フライトデータが空です');
        return;
      }

      // 初期データを取得（最初のフレーム用）
      const initialData = flight.data[0];
      console.log('初期フライトデータ：', initialData);
      console.log(`最高到達高度: ${flight.maxHeight.toFixed(2)}m, 最高速度: ${flight.maxSpeed.toFixed(2)}m/s, 最大水平距離: ${flight.maxDistance.toFixed(2)}m`);
      console.log(`最大フィンたわみ量: ${flight.maxFinDeflection.toFixed(4)}mm`);
      console.log(`姿勢安定性: 最大角度変化量=${flight.angleStability.maxAngleChangePerDt2.toFixed(2)}°`);

      // キーポイントを保存
      setKeyPoints(flight.keyPoints);

      setFlightData(flight.data);
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
          const timeIndex = Math.max(0, Math.min(Math.floor(elapsed / 0.02), flight.data.length - 1));

          // 追加のデバッグログ - 100フレームごとに状態を記録
          if (timeIndex % 100 === 0) {
            console.log(`Animation frame: time=${elapsed.toFixed(2)}s, index=${timeIndex}, total=${flight.data.length}`);
          }

          // データの安全性チェック
          if (timeIndex >= 0 && timeIndex < flight.data.length) {
            const currentData = flight.data[timeIndex];

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
              if (timeIndex < flight.data.length - 1) {
                const id = requestAnimationFrame(animate);
                setAnimationId(id);
              } else {
                // アニメーション終了 - 最終フレームに到達した時だけ結果を表示
                console.log('アニメーション完了: 最終インデックス到達');

                // シミュレーションデータから最大値を直接計算（状態変数に依存せず確実に取得）
                const maxHeight = Math.max(...flight.data.map(d => isNaN(d.height) ? 0 : d.height || 0));
                const maxSpeed = Math.max(...flight.data.map(d => isNaN(d.speedMagnitude) ? 0 :
                  Math.abs(d.speedMagnitude) || 0));
                const maxDistance = Math.max(...flight.data.map(d => isNaN(d.physicsX) ? 0 :
                  Math.abs(d.physicsX) || 0));
                const maxFinDeflection = Math.max(...flight.data.map(d => isNaN(d.finDeflection) ? 0 :
                  d.finDeflection || 0));

                // 飛行結果の評価（直接計算した最大値を使用）
                const isDivergenceOK = maxSpeed < flight.calculations.finDivergenceSpeed;
                const isFlutterOK = maxSpeed < flight.calculations.finFlutterSpeed;
                const maxDeflectionPercent = (maxFinDeflection / finHeight) * 100;
                const isDeflectionOK = maxDeflectionPercent <= 3;

                // 姿勢安定性の判定結果 - シミュレーション結果から直接取得
                const isAngleStableOK = flight.angleStability.isAngleStableOK;
                const maxAngleChangePerDt2 = flight.angleStability.maxAngleChangePerDt2;

                // 総合判定は全てのチェックをパスする必要がある
                const isOverallOK = isDivergenceOK && isFlutterOK && isDeflectionOK && isAngleStableOK;

                // 結果オブジェクトの作成
                const results = {
                  maxSpeed: maxSpeed,
                  maxHeight: maxHeight,
                  maxDistance: maxDistance,
                  maxFinDeflection: maxFinDeflection,
                  finHeight: finHeight,
                  maxDeflectionPercent: maxDeflectionPercent,
                  divergenceSpeed: formatSpeedValue(flight.calculations.finDivergenceSpeed),
                  flutterSpeed: formatSpeedValue(flight.calculations.finFlutterSpeed),
                  isDivergenceOK,
                  isFlutterOK,
                  isDeflectionOK,
                  isAngleStableOK,
                  maxAngleChangePerDt2,
                  isOverallOK,
                  launchAngle,
                  windSpeed,
                  windProfile
                };

                // 状態更新を一度に行う
                setFlightResults(results);
                setShowResultsPopup(true);

                // UI上の最大値表示も更新
                setCurrentMaxHeight(maxHeight);
                setCurrentMaxSpeed(maxSpeed);
                setCurrentMaxDistance(maxDistance);
                setCurrentMaxFinDeflection(maxFinDeflection);

                const completedFlightData = {
                  data: [...flight.data],
                  launchAngle,
                  windSpeed,
                  windProfile,
                  maxHeight: maxHeight,
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
          if (timeIndex < flight.data.length - 1) {
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
  }, [isLaunched, launchAngle, windSpeed, windProfile, simulationParams, handleReset, finHeight, trajectoryScale]);

  // パラメータ変更時のスケール更新
  useEffect(() => {
    if (isLaunched) return;

    // 風速プロファイルを引数として渡す
    const flight = calculateFlightPath(
      simulationParams,
      launchAngle,
      windSpeed,
      windProfile, // 風速プロファイルを追加
      {
        ...SVG_CONFIG,
        enhancedAttitudeControl, // 拡張姿勢制御フラグを渡す
        windAngleLimitation     // 風向きによる角度制限フラグを渡す
      }
    );

    if (flight && flight.maxHeight > 0) {
      const availableHeight = SVG_CONFIG.height - SVG_CONFIG.groundLevel;

      // 改良：より高いベース高さを設定
      const baseHeights = {
        '1/2A6-2': 100,
        'A8-3': 150,
        'B6-4': 200
      };

      const expectedBaseHeight = baseHeights[simulationParams.selectedMotor] || 150;
      const targetHeight = Math.max(flight.maxHeight * 1.3, expectedBaseHeight);

      const minHorizontalDistance = expectedBaseHeight * 0.9;
      const maxDistance = Math.max(flight.maxDistance || 0, minHorizontalDistance);

      // スケール計算
      const verticalScale = availableHeight / targetHeight;
      const horizontalScale = (SVG_CONFIG.width * 0.9) / (maxDistance * 2 || 1);

      const motorPowerFactor = {
        '1/2A6-2': 0.45,
        'A8-3': 0.35,
        'B6-4': 0.25
      };

      const powerFactor = motorPowerFactor[simulationParams.selectedMotor] || 0.35;

      // 最小/最大スケール値の調整
      const minScale = 10;
      const maxScale = 24;

      const rawScale = Math.min(verticalScale, horizontalScale) * powerFactor;
      // 最終スケールを調整 - 必ず最小スケールを適用
      const finalScale = Math.max(minScale, Math.min(maxScale, rawScale));

      // スケール設定
      setTrajectoryScale(finalScale);

      // ロケットスケールをさらに小さく
      const baseRocketScale = 0.03;
      setRocketScale(baseRocketScale * powerFactor);

      console.log(`パラメータ更新: モーター=${simulationParams.selectedMotor}, 高度=${flight.maxHeight.toFixed(1)}m, スケール=${finalScale.toFixed(2)}`);
    }
  }, [isLaunched, launchAngle, windSpeed, windProfile, simulationParams]);

  // 初期表示時の強制スケール設定用のuseEffect追加
  useEffect(() => {
    // コンポーネント初期マウント時に強制的にスケールを設定
    const initialScale = getInitialScaleForMotor(selectedMotor);
    const initialRocketScale = 0.03 * getMotorPowerFactor(selectedMotor);

    setTrajectoryScale(initialScale);
    setRocketScale(initialRocketScale);

    console.log(`初期表示: スケール=${initialScale}, ロケットスケール=${initialRocketScale}`);
  }, []); // 空の依存配列で初回のみ実行

  // クリーンアップ
  useEffect(() => {
    return () => {
      if (animationId) {
        cancelAnimationFrame(animationId);
      }
      if (sliderTimeoutRef.current) {
        clearTimeout(sliderTimeoutRef.current);
      }
    };
  }, [animationId]);

  // 本フックから公開する関数とパラメータ
  return {
    // デザインパラメータ
    noseShape, setNoseShape: setNoseShapeAndUpdate,
    noseHeight, setNoseHeight: debounce(setNoseHeight, SLIDER_DEBOUNCE_TIME),
    bodyHeight, setBodyHeight: debounce(setBodyHeight, SLIDER_DEBOUNCE_TIME),
    bodyWidth, setBodyWidth: debounce(setBodyWidth, SLIDER_DEBOUNCE_TIME),
    finHeight, setFinHeight: debounce(setFinHeight, SLIDER_DEBOUNCE_TIME),
    finBaseWidth, setFinBaseWidth: debounce(setFinBaseWidth, SLIDER_DEBOUNCE_TIME),
    finTipWidth, setFinTipWidth: debounce(setFinTipWidth, SLIDER_DEBOUNCE_TIME),
    finThickness, setFinThickness: debounce(setFinThickness, SLIDER_DEBOUNCE_TIME),
    finSweepLength, setFinSweepLength: debounce(setFinSweepLength, SLIDER_DEBOUNCE_TIME),
    finMaterial, setFinMaterial,
    finCount, setFinCount,

    // 分析パラメータ
    weight, setWeight: debounce(setWeight, SLIDER_DEBOUNCE_TIME),
    centerOfGravity, setCenterOfGravity: debounce(setCenterOfGravity, SLIDER_DEBOUNCE_TIME),
    selectedMotor, setSelectedMotor,
    selectedParachute, setSelectedParachute,

    // シミュレーションパラメータ
    launchAngle, setLaunchAngle: debounce(setLaunchAngle, SLIDER_DEBOUNCE_TIME),
    windSpeed, setWindSpeed: debounce(setWindSpeed, SLIDER_DEBOUNCE_TIME),
    windProfile, setWindProfile,
    showWindArrows, setShowWindArrows,

    // シミュレーション状態
    isLaunched, setIsLaunched,
    flightData, currentTime,
    currentHeight, currentSpeed, currentDistance, currentFinDeflection,
    currentMaxHeight, currentMaxSpeed, currentMaxDistance, currentMaxFinDeflection,
    completedFlights, keyPoints,

    // 表示設定
    design, analysis,
    trajectoryScale, rocketScale,

    // 新しい計算結果
    projectedAreas, volumes, pressureCenter, aerodynamicCenter, stabilityCenterOfPressure, staticMargins,

    // 計算結果
    calculations,

    // 初期化状態
    isInitialized,

    // スライダーのデータと更新関数
    noseHeightInputRef, handleNoseHeight: handleSliderChange(noseHeightInputRef, setNoseHeight),
    bodyHeightInputRef, handleBodyHeight: handleSliderChange(bodyHeightInputRef, setBodyHeight),
    bodyWidthInputRef, handleBodyWidth: handleSliderChange(bodyWidthInputRef, setBodyWidth),
    finHeightInputRef, handleFinHeight: handleSliderChange(finHeightInputRef, setFinHeight),
    finBaseWidthInputRef, handleFinBaseWidth: handleSliderChange(finBaseWidthInputRef, setFinBaseWidth),
    finTipWidthInputRef, handleFinTipWidth: handleSliderChange(finTipWidthInputRef, setFinTipWidth),
    finThicknessInputRef, handleFinThickness: handleSliderChange(finThicknessInputRef, setFinThickness),
    finSweepLengthInputRef, handleFinSweepLength: handleSliderChange(finSweepLengthInputRef, setFinSweepLength),
    weightInputRef, handleWeight: handleSliderChange(weightInputRef, setWeight),
    centerOfGravityInputRef, handleCenterOfGravity: handleSliderChange(centerOfGravityInputRef, setCenterOfGravity),
    launchAngleInputRef, handleLaunchAngle: handleSliderChange(launchAngleInputRef, setLaunchAngle),
    windSpeedInputRef, handleWindSpeed: handleSliderChange(windSpeedInputRef, setWindSpeed),

    // 描画関数 - 外部のレンダリング関数を呼び出すためのラッパー
    getNosePath: (config) => getNosePath(config, noseShape, noseHeight, bodyWidth, bodyHeight),
    getBodyPath: (config) => getBodyPath(config, bodyHeight, bodyWidth),
    getLeftFinPath: (config) => getLeftFinPath(config, bodyWidth, finHeight, finBaseWidth, finSweepLength, finTipWidth),
    getRightFinPath: (config) => getRightFinPath(config, bodyWidth, finHeight, finBaseWidth, finSweepLength, finTipWidth),
    getCenterFinsPath: (config) => getCenterFinsPath(config, finThickness, finBaseWidth, finSweepLength, finTipWidth),
    getParachutePath: getParachute,
    getParachuteStringPaths: getParachuteStrings,
    getWindArrow,
    getWindProfileArrows: getWindArrows,

    // 座標変換
    metersToSvgX: convertMetersToSvgX,
    metersToSvgY: convertMetersToSvgY,

    // viewBox計算
    getDesignViewBox: computeDesignViewBox,
    getAnalysisViewBox: computeAnalysisViewBox,

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

    // 操作関数
    handleLaunch,
    handleReset,

    // 結果表示関連
    showResultsPopup,
    flightResults,
    lastFlightResults,
    getCurrentFlightPhase,
    handleCloseResultsPopup
  };
};

// 統合されたロケットシミュレーターコンポーネント
const IntegratedRocketSimulator = () => {
  const [activeTab, setActiveTab] = useState(UI_CONFIG.defaultTab);
  const [debugView, setDebugView] = useState(false);
  // 開発モードのステートを定数から初期化し、キーボードショートカットでの切り替えを無効化
  const [devMode, setDevMode] = useState(ENABLE_DEV_MODE);

  // 開発モードが有効な場合のみコンソールにログを出力（オプション）
  useEffect(() => {
    if (ENABLE_DEV_MODE) {
      console.log('開発モードが有効です');
    }
  }, []);

  // ロケットシミュレーターフックを使用
  const rocketSim = useRocketSimulator();

  const navigate = useNavigate();
  // ログアウト処理
  const handleLogout = () => {
    localStorage.removeItem('isLoggedIn'); // ログイン状態を削除
    navigate('/login'); // ログイン画面にリダイレクト
  }

  return (
    <div className="relative max-w-6xl mx-auto p-4">
      <h2 className="text-2xl font-bold mb-4">AVIENTER_2D</h2>
      <button
        onClick={handleLogout}
        className="absolute top-4 right-4 px-4 py-2 border border-gray text-gray font-bold rounded hover:bg-white hover:text-black-300 transition z-10"
      >
        ログアウト
      </button>
      <div className="flex border-b mb-6">
        <button
          className={`px-6 py-3 font-medium ${activeTab === 'design' ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-700'} rounded-t-lg`}
          onClick={() => setActiveTab('design')}
        >
          形状設計
        </button>
        <button
          className={`px-6 py-3 font-medium ${activeTab === 'analysis' ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-700'} rounded-t-lg ml-1`}
          onClick={() => setActiveTab('analysis')}
        >
          重量・空力特性
        </button>
        <button
          className={`px-6 py-3 font-medium ${activeTab === 'simulation' ? 'bg-blue-500 text-white' : 'bg-gray-200 text-gray-700'} rounded-t-lg ml-1`}
          onClick={() => setActiveTab('simulation')}
        >
          飛行シミュレーション
        </button>
      </div>

      <div>
        {activeTab === 'design' && <DesignTab rocketSim={rocketSim} />}
        {activeTab === 'analysis' && <AnalysisTab rocketSim={rocketSim} getSafeValue={getSafeValue} />}
        {activeTab === 'simulation' && <SimulationTab rocketSim={rocketSim} debugView={debugView} setDebugView={setDebugView} devMode={devMode} />}
      </div>

      {/* 開発モード表示 - 開発モード時のみ表示 */}
      {devMode && (
        <div className="mt-2 py-1 px-2 bg-yellow-100 border border-yellow-300 rounded text-xs text-gray-700 flex items-center">
          <span className="mr-2 px-1 py-0.5 bg-yellow-300 rounded-sm text-xs font-bold">開発モード</span>
          <span>Ctrl+Shift+D でオン/オフ</span>
          <button
            className="ml-auto px-2 py-0.5 bg-gray-200 hover:bg-gray-300 rounded text-xs"
            onClick={() => setDebugView(!debugView)}
          >
            {debugView ? "デバッグ表示オフ" : "デバッグ表示オン"}
          </button>
        </div>
      )}

      {/* 著作権・免責事項フッター */}
      <div className="mt-8 pt-4 border-t border-gray-300 text-sm text-gray-600">
        <p className="mb-1">© 2025 AVIATOL - ご利用は個人での範囲に限ります</p>
        <p className="mb-1">許可がない場合、商用でのご利用はご遠慮ください</p>
        <p className="mb-1">紹介目的でのご利用はこの限りではありません</p>
        <p className="mb-1">ご紹介いただける場合は、info(a)aviatol.comまでご連絡をお願いします（(a)は@に変えてください）</p>
        <p className="mb-1">本シミュレーションは実機での動作を保証するものではありません</p>
        <p>ロケットの打上げは自身と周りの安全に十分配慮し、個人の責任の範囲で行なってください</p>
      </div>
    </div>
  );
};

export default IntegratedRocketSimulator;