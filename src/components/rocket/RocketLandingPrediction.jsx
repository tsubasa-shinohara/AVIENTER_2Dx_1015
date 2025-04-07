// 既存の風速計算関数をインポート
import { calculateWindSpeedAtHeight } from './RocketPhysics';

/**
 * シミュレーション結果から着地予測を計算するための関数
 * @param {Object} rocketParams - ロケットのパラメータ
 * @param {Array} flightData - シミュレーションの飛行データ配列
 * @param {number} windSpeed - 風速 (m/s)
 * @param {string} windProfile - 風速プロファイルの種類
 * @returns {Object} 着地予測情報
 */
export const predictLanding = (rocketParams, flightData, windSpeed, windProfile) => {
  console.log('predictLanding called with:', {
    rocketParams: rocketParams ? 'present' : 'missing',
    flightDataType: typeof flightData,
    flightDataIsArray: Array.isArray(flightData),
    windSpeed,
    windProfile
  });

  // データ構造の確認 - オブジェクトかつdata配列プロパティを持つ場合の対応
  let dataArray = flightData;

  // dataプロパティが存在する場合はそれを使用（オブジェクトとして渡された場合の対応）
  if (flightData && typeof flightData === 'object' && !Array.isArray(flightData) && flightData.data && Array.isArray(flightData.data)) {
    console.log('Using data property from flight data object');
    dataArray = flightData.data;
  }

  // 数値が渡された場合（エラーログより）
  if (typeof flightData === 'number') {
    console.error('Flight data is a number, which is invalid');
    return defaultLandingPrediction();
  }

  // 配列でない場合のチェック
  if (!Array.isArray(dataArray)) {
    console.error('Flight data is not an array:', typeof dataArray);
    return defaultLandingPrediction();
  }

  // 空配列のチェック
  if (dataArray.length === 0) {
    console.error('Flight data array is empty');
    return defaultLandingPrediction();
  }

  // rocketParamsのチェック
  if (!rocketParams || typeof rocketParams !== 'object') {
    console.error('Invalid rocket parameters, using default values');
    rocketParams = {
      weight: 50,
      bodyWidth: 31,
      bodyHeight: 255,
      selectedParachute: 'φ300'
    }; // デフォルト値
  }

  // シミュレーションの最終データポイントを取得
  const lastDataPoint = dataArray[dataArray.length - 1];

  // 最終データポイントの妥当性チェック
  if (!lastDataPoint) {
    console.error('Last data point is undefined');
    return defaultLandingPrediction();
  }

  // プロパティの存在確認と安全な取得
  const finalHeight = lastDataPoint.height ?? 0;      // 高度 (m)
  const finalX = lastDataPoint.physicsX ?? 0;         // 水平位置 (m)
  const finalVx = lastDataPoint.vx ?? 0;              // 水平方向速度 (m/s)
  const finalVy = lastDataPoint.vy ?? 0;              // 垂直方向速度 (m/s)
  const isParachuteActive = lastDataPoint.isParachuteActive ?? false;
  const isParachuteEjected = lastDataPoint.isParachuteEjected ?? false;

  // 高度がない、またはすでに着地している場合
  if (finalHeight <= 0) {
    return {
      landingX: finalX,
      landingDistance: Math.abs(finalX),
      timeToLanding: 0,
      isPrediction: false
    };
  }

  // パラシュートサイズを取得 (φ300 -> 300mm -> 0.3m)
  const parachuteDiameter = rocketParams.selectedParachute ?
    parseFloat(rocketParams.selectedParachute.replace('φ', '')) / 1000 : 0.3; // mm → m

  // ボディパラメータを取得
  const bodyDiameter = rocketParams.bodyWidth / 1000; // mm → m
  const bodyLength = rocketParams.bodyHeight / 1000;  // mm → m

  // 物理定数
  const g = 9.81;  // 重力加速度 (m/s²)
  const PI = Math.PI;
  const rho = 1.225; // 空気密度 (kg/m³)

  // ロケットの質量（グラムからキログラムに変換）
  const mass_kg = rocketParams.weight * 0.001; // kg
  console.log('Using rocket mass:', mass_kg, 'kg');

  // パラシュート展開済みの場合の降下速度（一定と仮定）
  // パラシュート降下時の終端速度
  const parachuteTerminalVelocity = isParachuteActive ? Math.abs(finalVy || 5.0) : 5.0; // m/s

  // 終端速度が極端に小さい場合のデフォルト値設定
  const safeTerminalVelocity = parachuteTerminalVelocity > 0.5 ? parachuteTerminalVelocity : 5.0;

  // 残りの降下時間を推定（パラシュート展開済みなら終端速度を使用）
  const timeToLanding = finalHeight / safeTerminalVelocity;

  // 降下中の水平方向の移動を計算
  // 高度に応じた風の影響を詳細に計算
  // 降下時間を小さな時間ステップに分割して計算
  const dt = 0.05; // 時間ステップを小さくして精度を向上 (s)
  let currentHeight = finalHeight;
  let currentX = finalX;
  let currentVx = finalVx;
  let currentVy = finalVy || -safeTerminalVelocity; // 垂直速度が不明の場合は終端速度を使用
  let remainingTime = timeToLanding;

  try {
    // 着地までの移動をシミュレーション
    while (currentHeight > 0 && remainingTime > 0) {
      // 現在の高度における風速を計算
      const effectiveWindSpeed = calculateWindSpeedAtHeight(windSpeed, currentHeight, windProfile);

      // 現在の速度の大きさ
      const velocity = Math.sqrt(currentVx * currentVx + currentVy * currentVy);

      // 力の初期化
      let Fx = 0;
      let Fy = 0;

      if (isParachuteActive) {
        // パラシュート完全展開時の計算
        // パラシュートの抗力計算
        const Cd = 0.775; // パラシュートの抗力係数
        const Area = PI * Math.pow(parachuteDiameter / 2, 2);
        const Dp = 0.5 * Cd * rho * velocity * velocity * Area;

        // 速度方向への抗力
        if (velocity > 0.001) {
          Fx = -Dp * (currentVx / velocity);
          Fy = -Dp * (currentVy / velocity);
        }

        // 横風の影響を追加
        const Cdw = 0.25; // 横風の抗力係数
        const S = parachuteDiameter * parachuteDiameter * 0.785; // パラシュートの投影面積
        const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
        Fx -= Dw; // 横風の影響を追加
      }
      else if (isParachuteEjected && !isParachuteActive) {
        // パラシュート展開中の計算
        // 基本的な重力
        Fy = -mass_kg * g;

        // 軽い空気抵抗
        if (velocity > 0.001) {
          const dragCoefficient = 0.1;
          Fx = -dragCoefficient * currentVx;
          Fy -= dragCoefficient * currentVy;
        }

        // 横風の影響を小さく追加
        const Cdw = 0.25; // 横風の抗力係数
        const S = bodyDiameter * bodyLength * 0.5; // 半分展開時の面積
        const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
        Fx -= Dw * 0.5; // 展開中なので横風の影響を半分に
      }
      else {
        // パラシュート未展開時の計算
        // 重力
        Fy = -mass_kg * g;

        // 本体の空気抵抗
        if (velocity > 0.001) {
          const dragCoefficient = 0.05;
          Fx = -dragCoefficient * currentVx;
          Fy -= dragCoefficient * currentVy;
        }

        // 最小限の横風の影響
        const Cdw = 0.1; // より小さい抗力係数
        const S = bodyDiameter * bodyLength * 0.3; // 小さい投影面積
        const Dw = 0.5 * Cdw * rho * Math.abs(effectiveWindSpeed) * effectiveWindSpeed * S;
        Fx -= Dw * 0.2; // 横風の影響をさらに小さく
      }

      // 重力の追加（パラシュート完全展開時以外）
      if (!isParachuteActive) {
        Fy -= mass_kg * g;
      }

      // 加速度計算
      const ax = Fx / mass_kg;
      const ay = Fy / mass_kg;

      // 速度の更新 (v = v0 + at)
      currentVx = currentVx + ax * dt;
      currentVy = currentVy + ay * dt;

      // パラシュート展開時は終端速度に制限
      if (isParachuteActive && currentVy < -safeTerminalVelocity) {
        currentVy = -safeTerminalVelocity;
      }

      // 位置の更新 (x = x0 + vt)
      currentX = currentX + currentVx * dt;
      currentHeight = currentHeight + currentVy * dt;

      // 高度が負にならないよう制限
      if (currentHeight < 0) {
        currentHeight = 0;
      }

      // 残り時間の更新
      remainingTime -= dt;
    }
  } catch (error) {
    console.error('Error in landing prediction calculation:', error);
    // エラー発生時も最終位置を返す
  }

  // 予測着地地点
  const predictedLandingX = currentX;
  console.log('Predicted landing point:', {
    x: predictedLandingX,
    distance: Math.abs(predictedLandingX),
    time: timeToLanding
  });

  return {
    landingX: predictedLandingX,
    landingDistance: Math.abs(predictedLandingX),
    timeToLanding: timeToLanding,
    isPrediction: true
  };
};

// デフォルトの着地予測情報を返す関数
function defaultLandingPrediction() {
  return {
    landingX: 0,
    landingDistance: 0,
    timeToLanding: 0,
    isPrediction: true
  };
}

/**
 * 着地予測を含むフライトパス計算のラッパー関数
 * @param {Function} calculateFlightPath - 元のフライトパス計算関数
 * @param {Object} rocketParams - ロケットパラメータ
 * @param {number} angle - 発射角度
 * @param {number} windSpeed - 風速
 * @param {string} windProfile - 風速プロファイル
 * @param {Object} config - 設定オブジェクト
 * @returns {Object} 着地予測を含む飛行シミュレーション結果
 */
export const calculateFlightPathWithLanding = (calculateFlightPath, rocketParams, angle, windSpeed, windProfile, config) => {
  try {
    console.log('calculateFlightPathWithLanding called with:', {
      rocketParamsExist: !!rocketParams,
      angle,
      windSpeed,
      windProfile
    });

    // 元のフライトパス計算を実行
    const flightResult = calculateFlightPath(rocketParams, angle, windSpeed, windProfile, config);

    // フライトデータの確認
    if (!flightResult) {
      console.error('Flight result is undefined or null');
      return { data: [], maxHeight: 0, maxSpeed: 0, maxDistance: 0 };
    }

    // rocketParamsとflightResult.dataを正しく渡す
    const landingPrediction = predictLanding(rocketParams, flightResult.data, windSpeed, windProfile);

    // 最終のデータポイントの時間を取得
    const lastTimePoint = flightResult.data && Array.isArray(flightResult.data) && flightResult.data.length > 0 ?
      flightResult.data[flightResult.data.length - 1].time : 0;

    // 最終高度が0かどうかをチェック（着地済みかどうか）
    const finalHeight = flightResult.data && Array.isArray(flightResult.data) && flightResult.data.length > 0 ?
      flightResult.data[flightResult.data.length - 1].height : 0;

    const hasLanded = finalHeight <= 0;

    // 予測滞空時間の計算
    let totalFlightTime = 0;

    if (hasLanded) {
      // 既に着地している場合はシミュレーション時間を使用
      totalFlightTime = lastTimePoint;
    } else {
      // シミュレーション最大時間（MAX_TIME）は通常20秒
      const MAX_TIME = 20;

      // 未着地の場合、シミュレーション時間+予測着地時間を使用
      // ただし、シミュレーション時間がMAX_TIMEに近い場合は、シミュレーションが時間制限で終了した可能性がある
      const isSimulationTimeLimit = Math.abs(lastTimePoint - MAX_TIME) < 0.1; // 0.1秒の誤差を許容

      if (isSimulationTimeLimit) {
        // シミュレーションが時間制限で終了した場合
        totalFlightTime = lastTimePoint + landingPrediction.timeToLanding;
      } else {
        // 通常の場合
        totalFlightTime = lastTimePoint + landingPrediction.timeToLanding;
      }
    }

    // 予測滞空時間を着地予測に追加
    const enhancedLandingPrediction = {
      ...landingPrediction,
      totalFlightTime: totalFlightTime // 予測滞空時間を追加
    };

    // 結果に着地予測を追加
    return {
      ...flightResult,
      landing: enhancedLandingPrediction,
      // キーポイントに着地予測を追加
      keyPoints: {
        ...flightResult.keyPoints,
        predictedLanding: {
          time: lastTimePoint + landingPrediction.timeToLanding,
          distance: landingPrediction.landingDistance,
          x: landingPrediction.landingX,
          totalFlightTime: totalFlightTime // 予測滞空時間をキーポイントにも追加
        }
      }
    };
  } catch (error) {
    console.error('Error in flight path calculation with landing prediction:', error);
    // エラーが発生した場合は元の結果を返す
    try {
      return calculateFlightPath(rocketParams, angle, windSpeed, windProfile, config);
    } catch (fallbackError) {
      console.error('Even fallback calculation failed:', fallbackError);
      return { data: [], maxHeight: 0, maxSpeed: 0, maxDistance: 0 };
    }
  }
};