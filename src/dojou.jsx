// メインのシミュレーターコンポーネント
import React, { useState, useCallback, useEffect, useMemo, useRef } from 'react';

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
  calculateFlightPath
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
  ParameterSlider, DesignTab, AnalysisTab
} from './RocketUIComponents';

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
  
  // 2. rocketSimの初期状態が完全に構築されてから計算や描画を行うための状態
  const [isInitialized, setIsInitialized] = useState(false);
  
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
    selectedParachute
  }), [
    noseShape, noseHeight, bodyHeight, bodyWidth, 
    finHeight, finBaseWidth, finTipWidth, finThickness, finSweepLength,
    finMaterial, weight, centerOfGravity, selectedMotor, selectedParachute
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
        rotation: launchAngle,
        isParachuteEjected: false,
        isParachuteActive: false,
        parachuteDeploymentProgress: 0,
        horizontalDistance: 0,
        finDeflection: 0
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
        finDeflection: 0
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
        finDeflection: 0
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
      effectiveWindSpeed: isNaN(currentData.effectiveWindSpeed) ? 0 : (currentData.effectiveWindSpeed || 0)
    };
  }, [isLaunched, flightData, currentTime, launchAngle]);

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
        SVG_CONFIG
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
                
                // 姿勢安定性の判定結果 - 最大角度偏差を使用しない
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
  }, [isLaunched, launchAngle, windSpeed, windProfile, simulationParams, handleReset, finHeight, trajectoryScale, ANIMATION_SPEED]);

  // パラメータ変更時のスケール更新
  useEffect(() => {
    if (isLaunched) return;

    // 風速プロファイルを引数として渡す
    const flight = calculateFlightPath(
      simulationParams, 
      launchAngle, 
      windSpeed, 
      windProfile, // 風速プロファイルを追加
      SVG_CONFIG
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

// シミュレーションタブコンポーネント
const SimulationTab = ({ rocketSim, debugView, setDebugView }) => {
  const position = rocketSim.getCurrentPosition();
  const windArrow = rocketSim.getWindArrow(rocketSim.windSpeed);

  // ロケットパラメータを表示用にスケーリング
  const rocketDisplayParams = {
    bodyWidth: rocketSim.bodyWidth * rocketSim.rocketScale,
    bodyHeight: rocketSim.bodyHeight * rocketSim.rocketScale,
    noseHeight: rocketSim.noseHeight * rocketSim.rocketScale,
    finHeight: rocketSim.finHeight * rocketSim.rocketScale,
    finBaseWidth: rocketSim.finBaseWidth * rocketSim.rocketScale,
    finTipWidth: rocketSim.finTipWidth * rocketSim.rocketScale,
    finThickness: rocketSim.finThickness * rocketSim.rocketScale,
    finSweepLength: rocketSim.finSweepLength * rocketSim.rocketScale
  };

  // 姿勢表示用のロケットスケールを自動計算
  // 円の表示エリアの大きさに基づいて計算
  const circleRadius = 90; // 円の有効半径（余白含む）
  const { totalHeight } = rocketSim.getActualRocketDimensions({ 
    height: 100, 
    centerX: 50 
  });

  // 円の半径を基準にスケールを計算
  // ロケットの全長が円の直径の約80%になるようにする
  const attitudeDisplayScale = (circleRadius * 1.6) / totalHeight;

  // 姿勢表示用のロケットパラメータ
  const attitudeRocketParams = {
    bodyWidth: rocketSim.bodyWidth * attitudeDisplayScale,
    bodyHeight: rocketSim.bodyHeight * attitudeDisplayScale,
    noseHeight: rocketSim.noseHeight * attitudeDisplayScale,
    finHeight: rocketSim.finHeight * attitudeDisplayScale,
    finBaseWidth: rocketSim.finBaseWidth * attitudeDisplayScale,
    finTipWidth: rocketSim.finTipWidth * attitudeDisplayScale,
    finThickness: rocketSim.finThickness * attitudeDisplayScale,
    finSweepLength: rocketSim.finSweepLength * attitudeDisplayScale
  };

  // 重心位置の計算（スケールに合わせて）
  const cogY = rocketSim.centerOfGravity * attitudeDisplayScale;

  // 重心位置に基づいて回転中心を計算
  // 重心位置はノーズ先端からの距離なので、本体高さから重心位置を引くことで底部からの距離を計算
  const cogToBottom = attitudeRocketParams.bodyHeight + attitudeRocketParams.noseHeight - cogY;

  return (
    <div className="bg-white rounded-lg shadow-lg p-6">
      <div className="mb-4">
        <h3 className="text-xl font-bold mb-2">飛行シミュレーション</h3>
        
        <div className="grid grid-cols-2 gap-6 mb-4">
          <div>
            <div className="mb-4">
              <label className="block mb-2">発射角度 (度)</label>
              <input
                ref={rocketSim.launchAngleInputRef}
                type="range"
                value={rocketSim.launchAngle}
                min="-30"
                max="30"
                step="1"
                disabled={rocketSim.isLaunched}
                {...rocketSim.handleLaunchAngle}
                className="w-full cursor-pointer"
              />
              <div className="flex justify-between">
                <span>-30°</span>
                <span className="font-medium">{rocketSim.launchAngle}°</span>
                <span>30°</span>
              </div>
            </div>
            
            <div className="mb-4">
              <label className="block mb-2">横風速度 (m/s)</label>
              <input
                ref={rocketSim.windSpeedInputRef}
                type="range"
                value={rocketSim.windSpeed}
                min="-8"
                max="8"
                step="0.1"
                disabled={rocketSim.isLaunched}
                {...rocketSim.handleWindSpeed}
                className="w-full cursor-pointer"
              />
              <div className="flex justify-between">
                <span>-8 m/s</span>
                <span className="font-medium">{rocketSim.windSpeed.toFixed(1)} m/s</span>
                <span>8 m/s</span>
              </div>
            </div>
            
            <div className="mt-4">
              <label className="block mb-2">風速プロファイル</label>
              <div className="flex items-center gap-4">
                <select
                  value={rocketSim.windProfile}
                  onChange={(e) => rocketSim.setWindProfile(e.target.value)}
                  disabled={rocketSim.isLaunched}
                  className="p-2 border border-gray-300 rounded w-48"
                >
                  {Object.entries(WIND_PROFILES).map(([key, profile]) => (
                    <option key={key} value={key}>
                      {profile.name}
                    </option>
                  ))}
                </select>
                
                <div className="flex items-center">
                  <input
                    type="checkbox"
                    id="showWindArrows"
                    checked={rocketSim.showWindArrows}
                    onChange={(e) => rocketSim.setShowWindArrows(e.target.checked)}
                    className="mr-2"
                  />
                  <label htmlFor="showWindArrows">風速分布表示</label>
                </div>
              </div>
              
              <div className="mt-2 text-sm text-gray-600">
                <p>高度が上がるにつれて風速が強くなります。地形によって変化率が異なります。</p>
                {rocketSim.windProfile !== 'uniform' && rocketSim.windSpeed !== 0 && (
                  <p className="mt-1">
                    <span className="font-medium">現在の設定:</span> 地上風速 {rocketSim.windSpeed.toFixed(1)} m/s、
                    高度80mでは約 {rocketSim.calculateWindSpeedAtHeight(rocketSim.windSpeed, 80, rocketSim.windProfile).toFixed(1)} m/s
                  </p>
                )}
              </div>
            </div>
          </div>
              
          <div className="grid grid-cols-2 gap-4 text-sm">
            <div className="space-y-2">
              <div>現在時刻: {rocketSim.currentTime.toFixed(2)} s</div>
              <div>現在高度: {rocketSim.currentHeight.toFixed(1)} m</div>
              <div>現在速度: {rocketSim.currentSpeed.toFixed(1)} m/s</div>
              <div>現在水平距離: {rocketSim.currentDistance.toFixed(1)} m</div>
              <div>フィンたわみ量: {rocketSim.formatFinDeflection(rocketSim.currentFinDeflection)}</div>
              {position.effectiveWindSpeed && (
                <div>現在実効風速: {position.effectiveWindSpeed.toFixed(1)} m/s</div>
              )}
            </div>
            <div className="space-y-2">
              <div>最高到達高度: {rocketSim.currentMaxHeight.toFixed(1)} m</div>
              <div>最高速度: {rocketSim.currentMaxSpeed.toFixed(1)} m/s</div>
              <div>最大水平距離: {rocketSim.currentMaxDistance.toFixed(1)} m</div>
              <div>最大フィンたわみ量: {rocketSim.currentMaxFinDeflection.toFixed(2)} mm</div>
            </div>
          </div>
        </div>
        
        <div className="flex justify-between mb-4">
          <div className="space-x-2">
            <button 
              onClick={rocketSim.handleLaunch}
              disabled={rocketSim.isLaunched}
              className="px-4 py-2 bg-red-500 text-white rounded hover:bg-red-600 disabled:opacity-50"
            >
              発射
            </button>
            <button 
              onClick={rocketSim.handleReset}
              className="px-4 py-2 bg-gray-500 text-white rounded hover:bg-gray-600"
            >
              リセット
            </button>
            <button
              onClick={() => setDebugView(!debugView)}
              className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
            >
              {debugView ? "通常表示" : "デバッグ表示"}
            </button>
            {rocketSim.completedFlights.length > 0 && (
              <span className="text-sm text-gray-600 ml-2 italic">
                {rocketSim.completedFlights.length}回の軌跡を表示中
              </span>
            )}
          </div>
        </div>
        
        {/* シミュレーション表示エリアを横並びに変更 */}
        <div className="flex gap-4">
          {/* 左側: 飛行軌跡表示 */}
          <div className="relative w-full bg-white rounded-lg border">
            <svg 
              width="100%" 
              height={600} 
              viewBox={`0 0 800 600`}
              style={{ background: 'white' }}
            >
              {/* グリッド線 - 常に表示するよう変更 */}
              <>
                {/* 横線 - 高度メモリ */}
                {Array.from({ length: 10 }).map((_, i) => {
                  const heightMeters = i * 10; // 10mごとに線を引く
                  const y = rocketSim.metersToSvgY(heightMeters);
                  
                  if (y >= 0 && y <= 550) {
                    return (
                      <g key={`grid-h-${i}`}>
                        <line 
                          x1={0} 
                          y1={y} 
                          x2={800} 
                          y2={y} 
                          stroke="#ddd" 
                          strokeWidth="1" 
                          strokeDasharray="5,5" 
                        />
                        <text x={5} y={y - 5} fontSize="12" fill="#666">
                          {heightMeters}m
                        </text>
                      </g>
                    );
                  }
                  return null;
                })}
                
                {/* 縦線 - 水平距離メモリ */}
                {Array.from({ length: 11 }).map((_, i) => {
                  const distanceMeters = (i - 5) * 10; // -50mから+50mまで
                  const x = rocketSim.metersToSvgX(distanceMeters);
                  
                  if (x >= 0 && x <= 800) {
                    return (
                      <g key={`grid-v-${i}`}>
                        <line 
                          x1={x} 
                          y1={0} 
                          x2={x} 
                          y2={600} 
                          stroke="#ddd" 
                          strokeWidth="1" 
                          strokeDasharray="5,5" 
                        />
                        {distanceMeters !== 0 && (
                          <text x={x + 5} y={550 + 15} fontSize="12" fill="#666">
                            {distanceMeters}m
                          </text>
                        )}
                      </g>
                    );
                  }
                  return null;
                })}
              </>
              
              {/* 風速の視覚的表示 */}
              {(debugView || rocketSim.windSpeed !== 0) && (
                <g>
                  <text 
                    x={20} 
                    y={30} 
                    fontSize="16" 
                    fill={rocketSim.windSpeed === 0 ? "#999" : "#333"}
                    fontWeight={Math.abs(rocketSim.windSpeed) > 2 ? "bold" : "normal"}
                  >
                    風速: {rocketSim.windSpeed.toFixed(1)} m/s
                  </text>
                
                  {/* 風向き矢印 - テキストに合わせて配置 */}
                  {rocketSim.windSpeed !== 0 && (
                    <g>
                      {/* 風速に合わせた矢印の取得 */}
                      {(() => {
                        const windArrow = rocketSim.getWindArrow(rocketSim.windSpeed);
                        return (
                          <>
                            {/* 矢印の線 */}
                            <path 
                              d={windArrow.line} 
                              stroke={windArrow.color}
                              strokeWidth={windArrow.strokeWidth} 
                              fill="none"
                            />
                            {/* 矢印の頭 */}
                            <path 
                              d={windArrow.head}
                              stroke={windArrow.color}
                              strokeWidth={windArrow.strokeWidth} 
                              fill="none"
                            />
                          </>
                        );
                      })()}
                    </g>
                  )}
                </g>
              )}
              
              {/* 風速分布の矢印表示 - 矢印の位置を修正、四角形を削除 */}
              {rocketSim.showWindArrows && rocketSim.windSpeed !== 0 && (
                <g>
                  <text x={30} y={100} fontSize="12" fill="#666" fontWeight="bold">
                    高度別風速
                  </text>
                  
                  {rocketSim.getWindProfileArrows(rocketSim.windSpeed, rocketSim.windProfile).map((arrow, index) => (
                    <g key={`wind-arrow-${index}`}>
                      <text x={30} y={arrow.y - 15} fontSize="10" fill="#666">
                        {arrow.height}m
                      </text>
                      <path
                        d={arrow.arrowPath.line}
                        stroke={arrow.arrowPath.color}
                        strokeWidth={arrow.arrowPath.strokeWidth}
                        fill="none"
                      />
                      <path
                        d={arrow.arrowPath.head}
                        stroke={arrow.arrowPath.color}
                        strokeWidth={arrow.arrowPath.strokeWidth}
                        fill="none"
                      />
                      <text x={arrow.arrowPath.textX} y={arrow.arrowPath.textY + 4} fontSize="10" fill="#666">
                        {arrow.windSpeed.toFixed(1)} m/s
                      </text>
                    </g>
                  ))}
                </g>
              )}
              
              {/* 地面 */}
              <rect 
                x={0} 
                y={550} 
                width={800} 
                height={50} 
                fill="#4b5563" 
              />
              
              {/* 発射台 */}
              <g transform={`rotate(${rocketSim.launchAngle} 400 550)`}>
                <line 
                  x1={380} 
                  y1={550} 
                  x2={420} 
                  y2={550} 
                  stroke="#374151" 
                  strokeWidth="4" 
                />
                <line 
                  x1={400} 
                  y1={550} 
                  x2={400} 
                  y2={550 - 0.65 * rocketSim.trajectoryScale} 
                  stroke="#374151" 
                  strokeWidth="2" 
                />
              </g>
              
              {/* 過去のフライト軌跡 - データ間引きを減らして密度を上げる */}
              {rocketSim.completedFlights.map((flight, index) => (
                <path
                  key={`completed-flight-${index}`}
                  d={`M 400 550 ` + 
                    flight.data
                      .filter((d, i) => i % 2 === 0) // データ間引きをさらに減らす
                      .map(d => {
                        const x = 400 + d.physicsX * flight.scale;
                        const y = 550 - d.physicsY * flight.scale;
                        return `L ${x} ${y}`;
                      })
                      .join(' ')}
                  stroke="rgba(107, 114, 128, 0.5)"
                  strokeWidth="1.5"
                  fill="none"
                  strokeDasharray="2 2"
                />
              ))}
              
              {/* 現在のフライト軌跡 - 色を鮮やかに、線を太く */}
              {rocketSim.isLaunched && rocketSim.flightData.length > 0 && (
                <path
                  d={`M 400 550 ` + 
                    rocketSim.flightData
                      .filter((d, i) => i <= Math.floor(rocketSim.currentTime / 0.02) && i % 2 === 0)
                      .map(d => {
                        // NaNチェックを追加
                        const x = isNaN(d.physicsX) ? 400 : 400 + d.physicsX * rocketSim.trajectoryScale;
                        const y = isNaN(d.physicsY) ? 550 : 550 - d.physicsY * rocketSim.trajectoryScale;
                        return `L ${x} ${y}`;
                      })
                      .join(' ')}
                  stroke="#2563eb"
                  strokeWidth="2.5"
                  fill="none"
                />
              )}
            </svg>
          </div>
          


<div className="w-1/4 bg-white rounded-lg border p-4 flex flex-col items-center">
  <h4 className="font-semibold mb-2">ロケット姿勢表示</h4>
  
  {/* 円形の窓 */}
  <div className="relative">
    <svg 
      width={200} 
      height={200} 
      viewBox="-100 -100 200 200"
      className="border-2 border-gray-300 rounded-full overflow-hidden bg-gray-50"
    >
      {/* 背景の円 */}
      <circle cx="0" cy="0" r="98" fill="#f0f0f0" />
      
      {/* 角度目盛り */}
      {[...Array(12)].map((_, i) => {
        const angle = i * 30;
        const radians = (angle - 90) * Math.PI / 180;
        const x1 = Math.cos(radians) * 80;
        const y1 = Math.sin(radians) * 80;
        const x2 = Math.cos(radians) * 90;
        const y2 = Math.sin(radians) * 90;
        
        return (
          <g key={`mark-${i}`}>
            <line 
              x1={x1} 
              y1={y1} 
              x2={x2} 
              y2={y2} 
              stroke="#999" 
              strokeWidth="1" 
            />
            {i % 3 === 0 && (
              <text 
                x={Math.cos(radians) * 70} 
                y={Math.sin(radians) * 70} 
                textAnchor="middle" 
                dominantBaseline="middle"
                fontSize="10"
                fill="#666"
              >
                {angle}°
              </text>
            )}
          </g>
        );
      })}
      
      {/* 水平線と垂直線 */}
      <line x1="-90" y1="0" x2="90" y2="0" stroke="#666" strokeWidth="1" strokeDasharray="3,2" />
      <line x1="0" y1="-90" x2="0" y2="90" stroke="#666" strokeWidth="1" strokeDasharray="3,2" />
      
      {/* ロケット - 姿勢角度を表示（重心を中心に回転） */}
      <g transform={`rotate(${isNaN(position.rotation) ? 0 : position.rotation})`}>
  {/* ロケット本体 - Y位置をずらして重心が原点に来るようにする */}
  <g transform={`translate(0, ${-cogToBottom})`}>
    {/* ボディ */}
    <rect
      x={-attitudeRocketParams.bodyWidth / 2}
      y={-attitudeRocketParams.bodyHeight}
      width={attitudeRocketParams.bodyWidth}
      height={attitudeRocketParams.bodyHeight}
      fill="#9CA3AF"
      stroke="#374151"
      strokeWidth="0.5"
    />
    
    {/* ノーズ - 直接パスを指定 */}
    <path
      d={`M ${-attitudeRocketParams.bodyWidth / 2} ${-attitudeRocketParams.bodyHeight} 
          L 0 ${-attitudeRocketParams.bodyHeight - attitudeRocketParams.noseHeight} 
          L ${attitudeRocketParams.bodyWidth / 2} ${-attitudeRocketParams.bodyHeight} Z`}
      fill="#D1D5DB"
      stroke="#374151"
      strokeWidth="0.5"
    />
    
    {/* 左フィン */}
    <path
      d={`M ${-attitudeRocketParams.bodyWidth / 2} ${-attitudeRocketParams.finBaseWidth}
          L ${-attitudeRocketParams.bodyWidth / 2} ${0}
          L ${-attitudeRocketParams.bodyWidth / 2 - attitudeRocketParams.finHeight} ${attitudeRocketParams.finSweepLength}
          L ${-attitudeRocketParams.bodyWidth / 2 - attitudeRocketParams.finHeight} ${attitudeRocketParams.finSweepLength - attitudeRocketParams.finTipWidth} Z`}
      fill="#6B7280"
      stroke="#374151"
      strokeWidth="0.5"
    />
    
    {/* 右フィン */}
    <path
      d={`M ${attitudeRocketParams.bodyWidth / 2} ${-attitudeRocketParams.finBaseWidth}
          L ${attitudeRocketParams.bodyWidth / 2} ${0}
          L ${attitudeRocketParams.bodyWidth / 2 + attitudeRocketParams.finHeight} ${attitudeRocketParams.finSweepLength}
          L ${attitudeRocketParams.bodyWidth / 2 + attitudeRocketParams.finHeight} ${attitudeRocketParams.finSweepLength - attitudeRocketParams.finTipWidth} Z`}
      fill="#6B7280"
      stroke="#374151"
      strokeWidth="0.5"
    />
    
    {/* センターフィン */}
    <rect
      x={-attitudeRocketParams.finThickness / 2}
      y={-attitudeRocketParams.finBaseWidth}
      width={attitudeRocketParams.finThickness}
      height={attitudeRocketParams.finBaseWidth + Math.max(0, attitudeRocketParams.finSweepLength)}
      fill="#6B7280"
      stroke="#374151"
      strokeWidth="0.5"
    />
    
    {/* 重心位置のマーカー */}
    <circle cx="0" cy="0" r="2" fill="red" />
  </g>
</g>
      
      {/* 現在の角度表示 */}
      <text 
        x="0" 
        y="-85" 
        textAnchor="middle" 
        fontSize="12" 
        fill="#333"
        fontWeight="bold"
      >
        {(position.rotation % 360).toFixed(1)}°
      </text>
    </svg>
            </div>
            
            {/* 姿勢に関する追加情報 */}
            <div className="mt-4 text-sm">
              <div className="grid grid-cols-2 gap-x-2 gap-y-1">
                <div className="font-semibold">飛行フェーズ:</div>
                <div>{rocketSim.getCurrentFlightPhase()}</div>
                
                <div className="font-semibold">発射角度:</div>
                <div>{rocketSim.launchAngle}°</div>
                
                <div className="font-semibold">現在角度:</div>
                <div>{(position.rotation % 360).toFixed(1)}°</div>
                
                <div className="font-semibold">角度変化:</div>
                <div>{(position.rotation - rocketSim.launchAngle).toFixed(1)}°</div>
                
                <div className="font-semibold">風向き:</div>
                <div>{rocketSim.windSpeed > 0 ? "右" : rocketSim.windSpeed < 0 ? "左" : "なし"}</div>
              </div>
            </div>
            
            {/* 前回の飛翔結果 */}
            {rocketSim.lastFlightResults && (
              <LastFlightResults results={rocketSim.lastFlightResults} />
            )}
          </div>
          
          {/* デバッグ表示 */}
          {debugView && (
            <WindAngleLimitVisualizer 
              position={position}
              windSpeed={rocketSim.windSpeed}
              currentFlightPhase={rocketSim.getCurrentFlightPhase()}
            />
          )}
        </div>
        </div>
        <div className="bg-white rounded-lg shadow-sm mt-4 p-4 border border-gray-200">
          <h4 className="font-semibold mb-2">使用パラメータ</h4>
          <div className="grid grid-cols-2 gap-4 text-sm">
            <ul className="list-disc pl-5">
              <li>重量: {rocketSim.weight} g</li>
              <li>モーター: {rocketSim.selectedMotor}</li>
              <li>パラシュート: {rocketSim.selectedParachute}</li>
              <li>ノーズ形状: {NOSE_SHAPES[rocketSim.noseShape].name}</li>
              <li>抗力係数: {NOSE_SHAPES[rocketSim.noseShape].cd.toFixed(2)}</li>
            </ul>
            <ul className="list-disc pl-5">
              <li>ロケット全長: {rocketSim.calculations.totalHeight} mm</li>
              <li>ボディ直径: {rocketSim.bodyWidth} mm</li>
              <li>フィン翼幅: {rocketSim.finHeight} mm</li>
              <li>フィン厚み: {rocketSim.finThickness.toFixed(1)} mm</li>
              <li>フィン材質: {FIN_MATERIALS[rocketSim.finMaterial].name}</li>
              <li>重心位置: {rocketSim.centerOfGravity} mm</li>
              <li>風速プロファイル: {WIND_PROFILES[rocketSim.windProfile].name}</li>
            </ul>
          </div>
        </div>
      </div>
  );
};

