// UI関連のコンポーネント
import React, { useState, useMemo } from 'react';
import { NOSE_SHAPES, FIN_MATERIALS, WIND_PROFILES, PARACHUTE_SIZES } from './RocketConstants';
import { formatFinDeflection, formatSpeedValue } from './RocketPhysics';
import {
  getNosePath, getBodyPath, getLeftFinPath, getRightFinPath, getCenterFinsPath,
  getParachutePath, getParachuteStringPaths, getWindArrow, getWindProfileArrows,
  metersToSvgX, metersToSvgY, getActualRocketDimensions, getSafeRotationTransform,
  getAnalysisViewBox, getDesignViewBox, getRocketTransform, getSafeValue,
  getFinPaths, getTriFinLeftRightPaths
} from './RocketRendering';

import { calculateWindSpeedAtHeight } from './RocketPhysics';

// 安全な値を取得するヘルパー関数
const getSafeNumber = (value, defaultValue = 0) => {
  // 値がnull、undefined、NaN、無限大の場合はデフォルト値を返す
  if (value === null || value === undefined || isNaN(value) || !isFinite(value)) {
    return defaultValue;
  }
  // 数値型でない場合、変換を試みる
  if (typeof value !== 'number') {
    const parsed = parseFloat(value);
    // 変換できなければデフォルト値を返す
    return isNaN(parsed) ? defaultValue : parsed;
  }
  return value;
};

// Export all components
export {
  getSafeNumber,
  WindAngleLimitVisualizer,
  ResultsPopup,
  LastFlightResults,
  ParameterSlider,
  DesignTab,
  AnalysisTab,
  SimulationTab
};

// 風見効果のデバッグビジュアライザ
const WindAngleLimitVisualizer = ({ position, windSpeed, currentFlightPhase }) => {
  // 角度をラジアンから度に変換
  const currentAngleDegrees = (getSafeNumber(position?.rotation, 0) % 360) || 0;

  // 風向きに応じた制限角度 - 修正
  // 風が右から左（負の風速）なら -90度に制限
  // 風が左から右（正の風速）なら +90度に制限
  const limitAngleDegrees = Math.sign(windSpeed || 0) * 90;

  // 風上に向かっているかどうか - 修正
  const isMovingUpwind = ((windSpeed < 0 && currentAngleDegrees < 0) ||
    (windSpeed > 0 && currentAngleDegrees > 0)) || false;

  // 制限が適用されているかどうか - 修正
  const isLimitApplied = isMovingUpwind &&
    ((windSpeed < 0 && currentAngleDegrees <= limitAngleDegrees) ||
      (windSpeed > 0 && currentAngleDegrees >= limitAngleDegrees));

  // 表示色を判定
  const statusColor = isLimitApplied ? "#e74c3c" : (isMovingUpwind ? "#f39c12" : "#2ecc71");

  return (
    <div className="bg-white border border-gray-200 rounded-lg p-3 mt-4">
      <h4 className="font-semibold mb-2">風向きによる角度制限</h4>

      <div className="grid grid-cols-2 gap-x-2 gap-y-1 text-sm">
        <div>風速:</div>
        <div>{(windSpeed || 0).toFixed(1)} m/s</div>

        <div>風向き:</div>
        <div>{windSpeed > 0 ? "左 → 右" : windSpeed < 0 ? "右 → 左" : "無風"}</div>

        <div>現在角度:</div>
        <div>{currentAngleDegrees.toFixed(1)}°</div>

        <div>制限角度:</div>
        <div>{windSpeed !== 0 ? `${limitAngleDegrees.toFixed(0)}°` : "なし"}</div>

        <div>風上へ向かう:</div>
        <div>{isMovingUpwind ? "はい" : "いいえ"}</div>

        <div>角度制限機能:</div>
        <div>{rocketSim.enhancedAttitudeControl && rocketSim.windAngleLimitation ? "有効" : "無効"}</div>

        <div>飛行フェーズ:</div>
        <div>{currentFlightPhase || "初期化中"}</div>
      </div>

      <div className="mt-3 p-2 rounded" style={{ backgroundColor: statusColor + "20" /* 透明度を持たせる */ }}>
        <div className="text-center font-medium" style={{ color: statusColor }}>
          {isLimitApplied ?
            "角度制限適用中" :
            (isMovingUpwind ? "風上へ向かっています" : "風上へ向かっていません")}
        </div>
      </div>

      {/* 角度の可視化 */}
      <div className="mt-3 relative h-20 border-t border-b border-gray-300">
        {/* 制限角度のマーク */}
        {windSpeed !== 0 && (
          <div
            className="absolute h-full border-l-2 border-red-500"
            style={{
              left: `${50 + limitAngleDegrees / 3.6}%`, /* -90°〜90°を0%〜100%にマッピング */
              borderLeftStyle: 'dashed'
            }}
          >
            <span className="absolute -ml-6 -mt-5 text-xs text-red-500 font-medium">
              {limitAngleDegrees}°
            </span>
          </div>
        )}

        {/* 現在角度のマーク */}
        <div
          className="absolute h-full border-l-2 border-blue-500"
          style={{
            left: `${50 + currentAngleDegrees / 3.6}%`, /* -90°〜90°を0%〜100%にマッピング */
          }}
        >
          <span className="absolute -ml-6 mt-20 text-xs text-blue-500 font-medium">
            {currentAngleDegrees.toFixed(1)}°
          </span>
          <div className="absolute -ml-1.5 mt-8 w-3 h-3 bg-blue-500 rounded-full"></div>
        </div>

        {/* 角度スケール */}
        <div className="absolute w-full h-1 bg-gray-200 top-1/2 transform -translate-y-1/2">
          {[-90, -45, 0, 45, 90].map(deg => (
            <div key={deg} className="absolute h-2 border-l border-gray-400" style={{ left: `${50 + deg / 3.6}%` }}>
              <span className="absolute -ml-2 mt-2 text-xs text-gray-500">{deg}°</span>
            </div>
          ))}
        </div>
      </div>

      <div className="mt-3 text-xs text-gray-600">
        <p>
          <span className="font-semibold">風向きと角度制限の関係:</span>
        </p>
        <ul className="list-disc pl-5 mt-1">
          <li>風が右から左（−風速）: 右向き（−角度）は<span className="font-medium">−90°</span>まで</li>
          <li>風が左から右（+風速）: 左向き（+角度）は<span className="font-medium">+90°</span>まで</li>
        </ul>
      </div>
    </div>
  );
};

// 結果ポップアップコンポーネント
const ResultsPopup = ({ results, onClose }) => {
  // エラーがある場合は特別な表示
  if (results.isError) {
    // エラータイプに応じたメッセージとスタイルを決定
    let errorTitle = "シミュレーションエラー";
    let errorClass = "text-red-600";

    switch (results.errorType) {
      case "flutter":
        errorTitle = "フィンフラッターエラー";
        break;
      case "divergence":
        errorTitle = "フィンダイバージェンスエラー";
        break;
      case "deflection":
        errorTitle = "フィンたわみエラー";
        break;
      case "torque_invalid":
      case "torque_exceeded":
        errorTitle = "トルク計算エラー";
        break;
      default:
        errorTitle = "シミュレーションエラー";
    }

    return (
      <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
        <div className="bg-white p-6 rounded-lg shadow-lg max-w-md w-full">
          <h3 className={`text-xl font-bold mb-4 ${errorClass}`}>{errorTitle}</h3>
          <p className="mb-4">{results.errorMessage}</p>

          {/* エラータイプに応じた追加情報 */}
          {['flutter', 'divergence'].includes(results.errorType) && (
            <div className="mb-4 p-3 bg-gray-100 rounded">
              <p>シミュレーション時間: {results.errorTime?.toFixed(2)}秒</p>
              <p>速度: {results.velocity?.toFixed(2)}m/s</p>
              <p>フィンフラッター速度: {results.finFlutterSpeed?.toFixed(2)}m/s</p>
              <p>フィンダイバージェンス速度: {results.finDivergenceSpeed?.toFixed(2)}m/s</p>
            </div>
          )}

          {results.errorType === 'deflection' && (
            <div className="mb-4 p-3 bg-gray-100 rounded">
              <p>シミュレーション時間: {results.errorTime?.toFixed(2)}秒</p>
              <p>速度: {results.velocity?.toFixed(2)}m/s</p>
              <p>たわみ量比率: {results.maxDeflectionPercent?.toFixed(1)}%</p>
              <p>許容限界: 15%</p>
            </div>
          )}

          <p className="mb-4 text-gray-600">
            シミュレーション条件: 発射角度 {results.launchAngle}°, 風速 {results.windSpeed}m/s
          </p>
          <button
            className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
            onClick={onClose}
          >
            閉じる
          </button>
        </div>
      </div>
    );
  }

  if (!results) return null;

  const {
    maxSpeed = 0,
    maxHeight = 0,
    maxDistance = 0,
    maxFinDeflection = 0,
    finHeight = 0,
    maxDeflectionPercent = 0,
    divergenceSpeed = 0,
    flutterSpeed = 0,
    isDivergenceOK = false,
    isFlutterOK = false,
    isDeflectionOK = false,
    isAngleStableOK = false,
    isAbsoluteAngleOK = true, // 絶対角度判定の結果（デフォルトはtrue）
    isOverallOK = false,
    maxAngleChangePerDt2 = 0,
    maxAbsoluteAngle = 0, // 最大絶対角度
    windProfile = 'uniform'
  } = results;

  return (
    <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
      <div className="bg-white rounded-lg p-6 max-w-md">
        <h3 className="text-xl font-bold mb-4">飛行結果判定</h3>

        <div className="mb-4">
          <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
              <span>フィンダイバージェンス:</span>
              <span className={`font-bold ${isDivergenceOK ? 'text-green-600' : 'text-red-600'}`}>
                {isDivergenceOK ? 'OK' : 'NG'}
              </span>
            </div>
            <div className="text-sm text-gray-600">
              最高速度: {maxSpeed.toFixed(1)} m/s / 限界速度: {divergenceSpeed}
            </div>
          </div>

          <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
              <span>フィンフラッター:</span>
              <span className={`font-bold ${isFlutterOK ? 'text-green-600' : 'text-red-600'}`}>
                {isFlutterOK ? 'OK' : 'NG'}
              </span>
            </div>
            <div className="text-sm text-gray-600">
              最高速度: {maxSpeed.toFixed(1)} m/s / 限界速度: {flutterSpeed}
            </div>
          </div>

          <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
              <span>フィンたわみ量:</span>
              <span className={`font-bold ${isDeflectionOK ? 'text-green-600' : 'text-red-600'}`}>
                {isDeflectionOK ? 'OK' : 'NG'}
              </span>
            </div>

            <div className="text-sm text-gray-600">
              最大たわみ量: {formatFinDeflection(maxFinDeflection)} ({maxDeflectionPercent.toFixed(2)}% of スパン)
              / 限界: 3%
            </div>
          </div>

          {/* 姿勢安定性の判定 - 角度変化量判定 */}
          <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
              <span>姿勢安定性(角度変化):</span>
              <span className={`font-bold ${isAngleStableOK ? 'text-green-600' : 'text-red-600'}`}>
                {isAngleStableOK ? 'OK' : 'NG'}
              </span>
            </div>
            <div className="text-sm text-gray-600">
              最大角度変化: {maxAngleChangePerDt2.toFixed(1)}° / 限界: ±45° (0.2秒間)
            </div>
          </div>

          {/* 絶対角度判定の追加 */}
          <div className="mb-2">
            <div className="flex justify-between items-center mb-1">
              <span>姿勢安定性(絶対角度):</span>
              <span className={`font-bold ${isAbsoluteAngleOK ? 'text-green-600' : 'text-red-600'}`}>
                {isAbsoluteAngleOK ? 'OK' : 'NG'}
              </span>
            </div>
            <div className="text-sm text-gray-600">
              最大絶対角度: {maxAbsoluteAngle?.toFixed(1) || 0}° / 限界: ±112.5°
            </div>
          </div>

          <div className="mt-4 pt-2 border-t border-gray-300">
            <div className="flex justify-between items-center">
              <span className="font-semibold">総合判定:</span>
              <span className={`font-bold text-lg ${isOverallOK ? 'text-green-600' : 'text-red-600'}`}>
                {isOverallOK ? 'OK' : 'NG'}
              </span>
            </div>
          </div>
        </div>

        <div className="border-t border-gray-300 pt-4">
          <h4 className="font-semibold mb-2">飛行データ</h4>
          <div className="grid grid-cols-2 gap-2 text-sm">
            <div>最高高度:</div>
            <div>{maxHeight.toFixed(1)} m</div>
            <div>最高速度:</div>
            <div>{maxSpeed.toFixed(1)} m/s</div>
            <div>最大水平距離:</div>
            <div>{maxDistance.toFixed(1)} m</div>
            <div>風速プロファイル:</div>
            <div>{(WIND_PROFILES[windProfile] || {}).name || windProfile}</div>
          </div>
        </div>

        <button
          onClick={onClose}
          className="mt-4 bg-blue-500 hover:bg-blue-600 text-white py-2 px-4 rounded"
        >
          閉じる
        </button>
      </div>
    </div>
  );
};

// 前回の飛翔結果表示コンポーネント
const LastFlightResults = ({ results }) => {
  if (!results) return null;

  const {
    maxSpeed = 0,
    maxHeight = 0,
    maxDistance = 0,
    isDivergenceOK = false,
    isFlutterOK = false,
    isDeflectionOK = false,
    isAngleStableOK = false,
    isAbsoluteAngleOK = true, // 絶対角度判定の結果
    isOverallOK = false,
    launchAngle = 0,
    windSpeed = 0,
    windProfile = 'uniform'
  } = results;

  return (
    <div className="bg-white border border-gray-200 rounded-lg p-3 mt-4">
      <h4 className="font-semibold mb-2">前回の飛翔結果</h4>

      <div className="grid grid-cols-2 gap-x-2 gap-y-1 text-sm">
        <div>発射角度:</div>
        <div>{launchAngle}°</div>

        <div>風速:</div>
        <div>{windSpeed.toFixed(1)} m/s</div>

        <div>風速プロファイル:</div>
        <div>{(WIND_PROFILES[windProfile] || {}).name || windProfile}</div>

        <div>最高高度:</div>
        <div>{maxHeight.toFixed(1)} m</div>

        <div>最高速度:</div>
        <div>{maxSpeed.toFixed(1)} m/s</div>

        <div>最大水平距離:</div>
        <div>{maxDistance.toFixed(1)} m</div>
      </div>

      <div className="mt-2 pt-2 border-t border-gray-300 grid grid-cols-2 gap-2">
        <div className="text-sm">
          <span>フィンダイバージェンス:</span>
          <span className={`ml-1 font-medium ${isDivergenceOK ? 'text-green-600' : 'text-red-600'}`}>
            {isDivergenceOK ? 'OK' : 'NG'}
          </span>
        </div>

        <div className="text-sm">
          <span>フィンフラッター:</span>
          <span className={`ml-1 font-medium ${isFlutterOK ? 'text-green-600' : 'text-red-600'}`}>
            {isFlutterOK ? 'OK' : 'NG'}
          </span>
        </div>

        <div className="text-sm">
          <span>フィンたわみ:</span>
          <span className={`ml-1 font-medium ${isDeflectionOK ? 'text-green-600' : 'text-red-600'}`}>
            {isDeflectionOK ? 'OK' : 'NG'}
          </span>
        </div>

        {/* 姿勢安定性の判定に関する表示を修正 - 角度変化と絶対角度を個別に表示 */}
        <div className="text-sm">
          <span>角度変化安定性:</span>
          <span className={`ml-1 font-medium ${isAngleStableOK ? 'text-green-600' : 'text-red-600'}`}>
            {isAngleStableOK ? 'OK' : 'NG'}
          </span>
        </div>

        {/* 絶対角度判定を追加 */}
        <div className="text-sm col-span-2">
          <span>絶対角度安定性:</span>
          <span className={`ml-1 font-medium ${isAbsoluteAngleOK ? 'text-green-600' : 'text-red-600'}`}>
            {isAbsoluteAngleOK ? 'OK' : 'NG'}
          </span>
        </div>
      </div>

      <div className="mt-2 text-center">
        <span className="font-semibold">総合判定:</span>
        <span className={`ml-2 font-bold ${isOverallOK ? 'text-green-600' : 'text-red-600'}`}>
          {isOverallOK ? 'OK' : 'NG'}
        </span>
      </div>
    </div>
  );
};

// スライダーコンポーネント（パラメータ共通）
const ParameterSlider = ({
  label, value, min, max, step, disabled,
  displayValue, unit = "", minLabel, maxLabel,
  inputRef, handleSlider
}) => {
  // 表示する値（フォーマット指定ある場合はそれを使用）
  const display = displayValue || value;

  return (
    <div className="mb-4">
      <div className="flex justify-between mb-1">
        <label>{label}:</label>
        <span className="font-medium">{display}{unit}</span>
      </div>
      <input
        ref={inputRef}
        type="range"
        min={min}
        max={max}
        step={step || 1}
        value={value}
        disabled={disabled}
        {...(handleSlider || {})}
        className="w-full mb-1 cursor-pointer"
      />
      {(minLabel || maxLabel) && (
        <div className="flex justify-between text-sm text-gray-600">
          <span>{minLabel || min}{unit}</span>
          <span>{maxLabel || max}{unit}</span>
        </div>
      )}
    </div>
  );
};

// デザインタブコンポーネント
const DesignTab = ({ rocketSim }) => {
  // 実際のロケット寸法を取得
  const defaultDimensions = { totalHeight: 0 };
  const dimensions = rocketSim?.getActualRocketDimensions ?
    rocketSim.getActualRocketDimensions(rocketSim.design || {}) :
    defaultDimensions;

  const { totalHeight = 0 } = dimensions;

  // rocketSimオブジェクトが適切に初期化されているか確認
  if (!rocketSim || !rocketSim.noseShape) {
    return (
      <div className="bg-white rounded-lg shadow-lg p-6 mb-4 flex justify-center items-center">
        <p className="text-gray-500">ロケットシミュレーションの読み込み中...</p>
      </div>
    );
  }
  return (
    <div className="bg-white rounded-lg shadow-lg p-6 mb-4">
      <div className="grid grid-cols-2 gap-6">
        <div>
          <h3 className="text-xl font-bold mb-4">形状設計</h3>

          <div className="mb-6">
            <h4 className="text-lg font-semibold mb-2">ノーズ設定</h4>
            <div className="flex space-x-4 mb-2">
              <label className="flex items-center">
                <input
                  type="radio"
                  value="cone"
                  checked={rocketSim.noseShape === "cone"}
                  onChange={() => rocketSim.setNoseShape("cone")}
                  className="mr-2"
                />
                円錐
              </label>
              <label className="flex items-center">
                <input
                  type="radio"
                  value="parabola"
                  checked={rocketSim.noseShape === "parabola"}
                  onChange={() => rocketSim.setNoseShape("parabola")}
                  className="mr-2"
                />
                放物線
              </label>
              <label className="flex items-center">
                <input
                  type="radio"
                  value="ogive"
                  checked={rocketSim.noseShape === "ogive"}
                  onChange={() => rocketSim.setNoseShape("ogive")}
                  className="mr-2"
                />
                オジブ
              </label>
            </div>

            <ParameterSlider
              label="長さ"
              value={rocketSim.noseHeight}
              min={25}
              max={150}
              inputRef={rocketSim.noseHeightInputRef}
              handleSlider={rocketSim.handleNoseHeight}
              unit="mm"
              minLabel="25"
              maxLabel="150"
            />
          </div>

          <div className="mb-6">
            <h4 className="text-lg font-semibold mb-2">ボディ設定</h4>
            <ParameterSlider
              label="長さ"
              value={rocketSim.bodyHeight}
              min={100}
              max={500}
              inputRef={rocketSim.bodyHeightInputRef}
              handleSlider={rocketSim.handleBodyHeight}
              unit="mm"
              minLabel="100"
              maxLabel="300"
            />

            <ParameterSlider
              label="直径"
              value={rocketSim.bodyWidth}
              min={24}
              max={50}
              inputRef={rocketSim.bodyWidthInputRef}
              handleSlider={rocketSim.handleBodyWidth}
              unit="mm"
              minLabel="24"
              maxLabel="50"
            />
          </div>

          <div>
            <h4 className="text-lg font-semibold mb-2">フィン設定</h4>
            {/* フィン枚数選択UI追加 */}
            <div className="mb-4">
              <h5 className="font-medium mb-2">フィン枚数:</h5>
              <div className="flex space-x-4 mb-2">
                <label className="flex items-center">
                  <input
                    type="radio"
                    value="4"
                    checked={rocketSim.finCount === 4}
                    onChange={() => rocketSim.setFinCount(4)}
                    className="mr-2"
                  />
                  4枚
                </label>

                {/* 3枚オプション - シンプルに無効化 */}
                <div className="flex items-center">
                  <label className="flex items-center">
                    <input
                      type="radio"
                      value="3"
                      checked={rocketSim.finCount === 3}
                      onChange={() => rocketSim.setFinCount(3)}
                      className="mr-2"
                    />
                    3枚
                  </label>


                </div>
              </div>
            </div>

            <ParameterSlider
              label="翼幅"
              value={rocketSim.finHeight}
              min={10}
              max={250}
              step={0.5}
              inputRef={rocketSim.finHeightInputRef}
              handleSlider={rocketSim.handleFinHeight}
              unit="mm"
              displayValue={rocketSim.finHeight.toFixed(1)}
            />

            <ParameterSlider
              label="翼付け根長さ"
              value={rocketSim.finBaseWidth}
              min={10}
              max={150}
              inputRef={rocketSim.finBaseWidthInputRef}
              handleSlider={rocketSim.handleFinBaseWidth}
              unit="mm"
            />

            <ParameterSlider
              label="翼端長"
              value={rocketSim.finTipWidth}
              min={0}
              max={75}
              inputRef={rocketSim.finTipWidthInputRef}
              handleSlider={rocketSim.handleFinTipWidth}
              unit="mm"
            />

            <ParameterSlider
              label="厚さ"
              value={rocketSim.finThickness}
              min={1}
              max={10}
              step={0.5}
              inputRef={rocketSim.finThicknessInputRef}
              handleSlider={rocketSim.handleFinThickness}
              unit="mm"
              displayValue={rocketSim.finThickness.toFixed(1)}
              minLabel="1.0"
              maxLabel="10.0"
            />

            <ParameterSlider
              label="前縁後退代"
              value={rocketSim.finSweepLength}
              min={-50}
              max={150}
              inputRef={rocketSim.finSweepLengthInputRef}
              handleSlider={rocketSim.handleFinSweepLength}
              unit="mm"
              minLabel="-50"
              maxLabel="150"
            />

            <div className="mt-4 text-sm text-gray-600">
              <p><strong className="font-semibold">機体全長:</strong> {totalHeight}mm</p>
              <p>（ノーズ+ボディ+フィン後端部分）</p>
            </div>
          </div>
        </div>

        {/* ロケット表示部分 */}
        <div className="flex flex-col items-center justify-center bg-gray-50 rounded-lg p-4 border border-gray-200">
          <svg
            width={rocketSim.design?.width || 400}
            height={rocketSim.design?.height || 600}
            viewBox={rocketSim.getDesignViewBox ? rocketSim.getDesignViewBox() : "0 0 200 500"}
            className="overflow-visible"
          >
            {/* 3枚フィンの場合は先に左右フィンを描画してボディの後ろに表示 */}
            {rocketSim.finCount === 3 && (
              <>
                <path
                  d={getTriFinLeftRightPaths(
                    rocketSim.design || {},
                    rocketSim.bodyWidth,
                    rocketSim.finHeight,
                    rocketSim.finBaseWidth,
                    rocketSim.finSweepLength,
                    rocketSim.finTipWidth
                  ).leftFin}
                  fill="#6B7280" stroke="#374151"
                />
                <path
                  d={getTriFinLeftRightPaths(
                    rocketSim.design || {},
                    rocketSim.bodyWidth,
                    rocketSim.finHeight,
                    rocketSim.finBaseWidth,
                    rocketSim.finSweepLength,
                    rocketSim.finTipWidth
                  ).rightFin}
                  fill="#6B7280" stroke="#374151"
                />
              </>
            )}

            {/* 次にボディを描画 */}
            <path d={rocketSim.getBodyPath ? rocketSim.getBodyPath(rocketSim.design || {}) : ""} fill="#9CA3AF" stroke="#374151" />

            {/* 中央フィンを描画 */}
            <path d={rocketSim.getCenterFinsPath ? rocketSim.getCenterFinsPath(rocketSim.design || {}) : ""} fill="#6B7280" stroke="#374151" />

            {/* 4枚フィンの場合は左右フィンを中央フィンの後に描画 */}
            {rocketSim.finCount === 4 && (
              <>
                <path d={rocketSim.getLeftFinPath ? rocketSim.getLeftFinPath(rocketSim.design || {}) : ""} fill="#6B7280" stroke="#374151" />
                <path d={rocketSim.getRightFinPath ? rocketSim.getRightFinPath(rocketSim.design || {}) : ""} fill="#6B7280" stroke="#374151" />
              </>
            )}

            {/* 最後にノーズを描画 */}
            <path d={rocketSim.getNosePath ? rocketSim.getNosePath(rocketSim.design || {}) : ""} fill="#D1D5DB" stroke="#374151" />
          </svg>
        </div>
      </div>
    </div>
  );
};

// 分析タブコンポーネント
const AnalysisTab = ({ rocketSim, getSafeValue }) => {
  // rocketSimオブジェクトが適切に初期化されているか確認
  if (!rocketSim || !rocketSim.getActualRocketDimensions) {
    return (
      <div className="bg-white rounded-lg shadow-lg p-6 mb-4 flex justify-center items-center">
        <p className="text-gray-500">分析データの読み込み中...</p>
      </div>
    );
  }

  // ロケットの寸法を取得
  const defaultDimensions = { actualRocketHeight: 0, totalHeight: 0 };
  const dimensions = rocketSim.getActualRocketDimensions(rocketSim.analysis || {}) || defaultDimensions;
  const { actualRocketHeight = 0, totalHeight = 0 } = dimensions;

  // パラメータを安全に取得
  const noseHeight = getSafeNumber(rocketSim.noseHeight, 50);
  const bodyHeight = getSafeNumber(rocketSim.bodyHeight, 200);
  const bodyWidth = getSafeNumber(rocketSim.bodyWidth, 30);
  const finHeight = getSafeNumber(rocketSim.finHeight, 40);
  const finBaseWidth = getSafeNumber(rocketSim.finBaseWidth, 30);
  const finTipWidth = getSafeNumber(rocketSim.finTipWidth, 20);
  const finSweepLength = getSafeNumber(rocketSim.finSweepLength, 0);
  const finThickness = getSafeNumber(rocketSim.finThickness, 2);
  const centerOfGravity = getSafeNumber(rocketSim.centerOfGravity, totalHeight * 0.7);

  // フィンの後端部分の計算を修正
  const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);

  // 全長はノーズ + ボディ + フィン後端部分（フィン後端がボディ後端より後ろに出る場合）
  const totalRocketLength = noseHeight + bodyHeight + finExtension;

  // 重心位置スライダーの最大値は全長と同じ
  const maxCGPosition = totalRocketLength;

  // 安全に計算値を取得
  const safeCalculations = rocketSim.calculations || {};

  return (
    <div className="bg-white rounded-lg shadow-lg p-6 mb-4">
      <div className="grid grid-cols-2 gap-6">
        {/* 左側: 入力コントロールと設計値 */}
        <div>
          <h3 className="text-xl font-bold mb-4">重量・空力特性</h3>

          <ParameterSlider
            label="機体総重量"
            value={getSafeNumber(rocketSim.weight, 0)}
            min={15}
            max={150}
            step={0.1}
            inputRef={rocketSim.weightInputRef}
            handleSlider={rocketSim.handleWeight}
            unit="g"
            displayValue={(getSafeNumber(rocketSim.weight, 0)).toFixed(1)}
            minLabel="15"
            maxLabel="150"
          />

          <div className="mb-6">
            <div className="flex items-center mb-2">
              <span className="mr-2">重心位置:</span>
              <div className="flex-1">
                <input
                  ref={rocketSim.centerOfGravityInputRef}
                  type="range"
                  value={centerOfGravity}
                  min={0}
                  max={maxCGPosition}
                  step={1}
                  {...(rocketSim.handleCenterOfGravity || {})}
                  className="w-full cursor-pointer"
                />
              </div>
              <span className="ml-2 min-w-[60px] text-right">{centerOfGravity}mm</span>
            </div>
            <div className="flex justify-between text-sm text-gray-600">
              <span>ノーズ側</span>
              <span>エンジン側</span>
            </div>
          </div>

          <div className="mb-4">
            <h4 className="text-lg font-semibold mb-2">フィン材質</h4>
            <select
              value={rocketSim.finMaterial || 'balsa'}
              onChange={(e) => rocketSim.setFinMaterial && rocketSim.setFinMaterial(e.target.value)}
              className="w-full p-2 border border-gray-300 rounded mb-4"
            >
              <option value="light_balsa">軽量バルサ</option>
              <option value="balsa">バルサ</option>
              <option value="light_veneer">軽量ベニア</option>
            </select>
          </div>

          <div className="mb-4">
            <h4 className="text-lg font-semibold mb-2">モーター選択</h4>
            <select
              value={rocketSim.selectedMotor || 'A8-3'}
              onChange={(e) => rocketSim.setSelectedMotor && rocketSim.setSelectedMotor(e.target.value)}
              className="w-full p-2 border border-gray-300 rounded mb-4"
            >
              <option value="1/2A6-2">1/2A6-2</option>
              <option value="A8-3">A8-3</option>
              <option value="B6-4">B6-4</option>
            </select>
          </div>

          <div className="mb-6">
            <h4 className="text-lg font-semibold mb-2">パラシュート選択</h4>
            <select
              value={rocketSim.selectedParachute || 'φ300'}
              onChange={(e) => rocketSim.setSelectedParachute && rocketSim.setSelectedParachute(e.target.value)}
              className="w-full p-2 border border-gray-300 rounded"
            >
              <option value="φ180">φ180</option>
              <option value="φ250">φ250</option>
              <option value="φ300">φ300</option>
              <option value="φ600">φ600</option>
              <option value="φ900">φ900</option>
            </select>
          </div>

          <div className="space-y-2 bg-gray-50 p-4 rounded-lg">
            <h4 className="text-lg font-semibold mb-2">設計値</h4>
            <p><strong className="font-semibold">機体全長:</strong> {safeCalculations.totalHeight || 0} mm</p>
            <p><strong className="font-semibold">空力中心位置:</strong> {safeCalculations.aerodynamicCenter || 0} mm</p>
            <p><strong className="font-semibold">圧力中心位置:</strong> {safeCalculations.pressureCenter || 0} mm</p>
            <p><strong className="font-semibold">静安定用CP:</strong> {safeCalculations.stabilityCenterOfPressure || 0} mm</p>
            <p><strong className="font-semibold">静安定マージン:</strong> {safeCalculations.stabilityStaticMargin || 0}</p>
            <p><strong className="font-semibold">フィンダイバージェンス速度:</strong> {safeCalculations.finDivergenceSpeedDisplay || "計算中..."}</p>
            <p><strong className="font-semibold">フィンフラッター速度:</strong> {safeCalculations.finFlutterSpeedDisplay || "計算中..."}</p>
          </div>
        </div>

        {/* 右側: ロケット表示 - サイズを調整可能に変更 */}
        <div className="flex justify-center items-center">
          <div className="relative h-full flex flex-col items-center justify-center">
            {/* 凡例を上部に配置 */}
            <div className="bg-white border border-gray-200 rounded-lg px-4 py-2 shadow-sm mb-6 flex items-center space-x-6">
              <div className="flex items-center">
                <div className="w-10 h-0.5 bg-red-500 mr-2"></div>
                <span className="text-sm font-medium">重心 (CG)</span>
              </div>
              <div className="flex items-center">
                <div className="w-10 h-0.5 mr-2" style={{ borderTop: '2px dashed #3B82F6' }}></div>
                <span className="text-sm font-medium">圧力中心 (CP)</span>
              </div>
              <div className="flex items-center">
                <div className="w-10 h-0.5 mr-2" style={{ borderTop: '2px dotted #10B981' }}></div>
                <span className="text-sm font-medium">静安定用CP</span>
              </div>
            </div>

            {/* ロケット表示エリア - 縦向きロケットを中央に配置 */}
            <svg
              width={400}
              height={650}
              viewBox="-200 -355 400 915"
              className="border border-gray-200 bg-gray-50 rounded-lg"
            >
              {/* ロケットの全長を考慮して中央に配置 */}
              <g transform={`translate(0, ${totalRocketLength + 100})`}>
                {/* ロケットを全長に基づいて中央に配置するための計算と変換 */}
                <g transform={`translate(0, ${-totalRocketLength / 2 - 50})`}>
                  {/* 3枚フィンの場合は先に左右フィンを描画してボディの後ろに表示 */}
                  {rocketSim.finCount === 3 && (
                    <>
                      {/* 左フィン - 位置と幅を調整 */}
                      <path
                        d={`M ${-(bodyWidth / 2) * 1.73 / 2} ${-finBaseWidth}
                L ${-(bodyWidth / 2) * 1.73 / 2} ${0}
                L ${-(bodyWidth / 2) * 1.73 / 2 - finHeight * 1.73 / 2} ${-finBaseWidth + finSweepLength + finTipWidth}
                L ${-(bodyWidth / 2) * 1.73 / 2 - finHeight * 1.73 / 2} ${-finBaseWidth + finSweepLength} Z`}
                        fill="#6B7280"
                        stroke="#374151"
                        strokeWidth="1.5"
                      />

                      {/* 右フィン - 位置と幅を調整 */}
                      <path
                        d={`M ${(bodyWidth / 2) * 1.73 / 2} ${-finBaseWidth}
                L ${(bodyWidth / 2) * 1.73 / 2} ${0}
                L ${(bodyWidth / 2) * 1.73 / 2 + finHeight * 1.73 / 2} ${-finBaseWidth + finSweepLength + finTipWidth}
                L ${(bodyWidth / 2) * 1.73 / 2 + finHeight * 1.73 / 2} ${-finBaseWidth + finSweepLength} Z`}
                        fill="#6B7280"
                        stroke="#374151"
                        strokeWidth="1.5"
                      />
                    </>
                  )}

                  {/* ボディ - 座標系を底部(0,0)として上方向に負のy座標 */}
                  <rect
                    x={-(getSafeNumber(rocketSim.bodyWidth, 30) / 2)}
                    y={-getSafeNumber(rocketSim.bodyHeight, 200)}
                    width={getSafeNumber(rocketSim.bodyWidth, 30)}
                    height={getSafeNumber(rocketSim.bodyHeight, 200)}
                    fill="#9CA3AF"
                    stroke="#374151"
                    strokeWidth="1.5"
                  />

                  {/* ノーズ - 選択した形状タイプに基づいて描画 */}
                  {(() => {
                    const baseX1 = -(bodyWidth / 2);
                    const baseX2 = bodyWidth / 2;
                    const baseY = -bodyHeight;
                    const topX = 0;
                    const topY = baseY - noseHeight;

                    let nosePath;
                    if (rocketSim.noseShape === 'cone') {
                      // 円錐ノーズ
                      nosePath = `M ${baseX1} ${baseY} L ${topX} ${topY} L ${baseX2} ${baseY} Z`;
                    } else if (rocketSim.noseShape === 'parabola') {
                      // 放物線ノーズ
                      const controlY = topY + noseHeight * 0.15;
                      nosePath = `M ${baseX1} ${baseY} 
                      C ${baseX1} ${controlY}, ${topX} ${topY}, ${topX} ${topY}
                      C ${topX} ${topY}, ${baseX2} ${controlY}, ${baseX2} ${baseY} Z`;
                    } else {
                      // オジブノーズ (デフォルト)
                      const halfWidth = bodyWidth / 2;
                      nosePath = `M ${baseX1} ${baseY}
                      Q ${baseX1 + halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${topX} ${topY}
                      Q ${baseX2 - halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${baseX2} ${baseY} Z`;
                    }

                    return (
                      <path
                        d={nosePath}
                        fill="#D1D5DB"
                        stroke="#374151"
                        strokeWidth="1.5"
                      />
                    );
                  })()}

                  {/* 4枚フィンの場合のみ、左右フィンを表示 */}
                  {rocketSim.finCount === 4 && (
                    <>
                      {/* 左フィン */}
                      <path
                        d={`M ${-(bodyWidth / 2)} ${-finBaseWidth}
                L ${-(bodyWidth / 2)} ${0}
                L ${-(bodyWidth / 2) - finHeight} ${-finBaseWidth + finSweepLength + finTipWidth}
                L ${-(bodyWidth / 2) - finHeight} ${-finBaseWidth + finSweepLength} Z`}
                        fill="#6B7280"
                        stroke="#374151"
                        strokeWidth="1.5"
                      />

                      {/* 右フィン */}
                      <path
                        d={`M ${(bodyWidth / 2)} ${-finBaseWidth}
                L ${(bodyWidth / 2)} ${0}
                L ${(bodyWidth / 2) + finHeight} ${-finBaseWidth + finSweepLength + finTipWidth}
                L ${(bodyWidth / 2) + finHeight} ${-finBaseWidth + finSweepLength} Z`}
                        fill="#6B7280"
                        stroke="#374151"
                        strokeWidth="1.5"
                      />
                    </>
                  )}

                  {/* センターフィン - 共通で表示 */}
                  <rect
                    x={-(finThickness / 2)}
                    y={finSweepLength >= 0
                      ? -finBaseWidth
                      : -finBaseWidth - Math.abs(finSweepLength)}
                    width={finThickness}
                    height={finSweepLength >= 0
                      ? finBaseWidth + Math.max(0, finSweepLength + finTipWidth - finBaseWidth)
                      : finBaseWidth + Math.abs(finSweepLength)}
                    fill="#6B7280"
                    stroke="#374151"
                    strokeWidth="1.5"
                  />

                  {/* 重心マーカー - ノーズ先端からの距離を座標系に変換 */}
                  <line
                    x1={-60}
                    y1={-(bodyHeight + noseHeight) + centerOfGravity}
                    x2={60}
                    y2={-(bodyHeight + noseHeight) + centerOfGravity}
                    stroke="#EF4444"
                    strokeWidth="2.5"
                  />

                  {/* 圧力中心マーカー */}
                  <line
                    x1={-60}
                    y1={-(getSafeNumber(rocketSim.bodyHeight, 200) + getSafeNumber(rocketSim.noseHeight, 50)) + Math.min(
                      totalRocketLength,
                      Math.max(0, getSafeNumber(safeCalculations.pressureCenter, 0))
                    )}
                    x2={60}
                    y2={-(getSafeNumber(rocketSim.bodyHeight, 200) + getSafeNumber(rocketSim.noseHeight, 50)) + Math.min(
                      totalRocketLength,
                      Math.max(0, getSafeNumber(safeCalculations.pressureCenter, 0))
                    )}
                    stroke="#3B82F6"
                    strokeWidth="2"
                    strokeDasharray="6,3"
                  />

                  {/* 静安定マージン用圧力中心マーカー */}
                  <line
                    x1={-60}
                    y1={-(getSafeNumber(rocketSim.bodyHeight, 200) + getSafeNumber(rocketSim.noseHeight, 50)) + Math.min(
                      totalRocketLength,
                      Math.max(0, getSafeNumber(safeCalculations.stabilityCenterOfPressure, 0))
                    )}
                    x2={60}
                    y2={-(getSafeNumber(rocketSim.bodyHeight, 200) + getSafeNumber(rocketSim.noseHeight, 50)) + Math.min(
                      totalRocketLength,
                      Math.max(0, getSafeNumber(safeCalculations.stabilityCenterOfPressure, 0))
                    )}
                    stroke="#10B981"
                    strokeWidth="2"
                    strokeDasharray="3,3"
                  />
                </g>
              </g>
            </svg>
          </div>
        </div>
      </div>
    </div>
  );
};

// シミュレーションタブコンポーネント - 修正版
const SimulationTab = ({ rocketSim, debugView, setDebugView, devMode = false }) => {
  const position = rocketSim.getCurrentPosition();
  const windArrow = rocketSim.getWindArrow(rocketSim.windSpeed);

  // 姿勢表示用のロケットスケール計算の改良
  const calculateAttitudeDisplayScale = (rocketParams, circleRadius = 90) => {
    try {
      // 必要なパラメータを安全に取得
      const noseHeight = getSafeNumber(rocketParams.noseHeight, 50);
      const bodyHeight = getSafeNumber(rocketParams.bodyHeight, 200);
      const finSweepLength = getSafeNumber(rocketParams.finSweepLength, 0);
      const finTipWidth = getSafeNumber(rocketParams.finTipWidth, 20);
      const finBaseWidth = getSafeNumber(rocketParams.finBaseWidth, 30);
      const finHeight = getSafeNumber(rocketParams.finHeight, 40);
      const bodyWidth = getSafeNumber(rocketParams.bodyWidth, 31);

      // ロケットの寸法を計算
      const actualRocketHeight = noseHeight + bodyHeight;

      // フィンの後端がボディより出る場合を計算
      const finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);

      // 横方向の最大幅を計算（フィン含む）
      const totalWidth = bodyWidth + (finHeight * 2);

      // 全長は機体長 + フィン後端部分
      const totalHeight = actualRocketHeight + finExtension;

      // より小さいスケールファクターを使用して確実に表示窓に収まるようにする
      // 余裕を持たせて80%のサイズに制限
      const heightScale = (circleRadius * 0.8) / totalHeight;
      const widthScale = (circleRadius * 0.8) / totalWidth;

      // 小さい方のスケールを採用
      const scale = Math.min(heightScale, widthScale);

      // 最大/最小スケールを制限
      return Math.max(0.03, Math.min(0.25, scale));
    } catch (error) {
      console.error('姿勢表示スケール計算エラー:', error);
      return 0.1; // エラー時のデフォルト値を小さく設定
    }
  };

  // 姿勢表示用のロケットパラメータ計算の改良
  const attitudeDisplayScale = calculateAttitudeDisplayScale(rocketSim);

  // スケールに合わせてパラメータを変換
  const attitudeRocketParams = {
    noseShape: rocketSim.noseShape,
    bodyWidth: rocketSim.bodyWidth * attitudeDisplayScale,
    bodyHeight: rocketSim.bodyHeight * attitudeDisplayScale,
    noseHeight: rocketSim.noseHeight * attitudeDisplayScale,
    finHeight: rocketSim.finHeight * attitudeDisplayScale,
    finBaseWidth: rocketSim.finBaseWidth * attitudeDisplayScale,
    finTipWidth: rocketSim.finTipWidth * attitudeDisplayScale,
    finThickness: rocketSim.finThickness * attitudeDisplayScale,
    finSweepLength: rocketSim.finSweepLength * attitudeDisplayScale
  };

  // 実際のロケット全長（スケール適用後）
  const scaledRocketHeight = (rocketSim.noseHeight + rocketSim.bodyHeight) * attitudeDisplayScale;
  const scaledRocketNoseHeight = rocketSim.noseHeight * attitudeDisplayScale;
  const scaledRocketBodyHeight = rocketSim.bodyHeight * attitudeDisplayScale;

  // 重心位置（スケール適用後）- ノーズ先端からの距離
  const scaledCogY = rocketSim.centerOfGravity * attitudeDisplayScale;

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

  // 重心位置の計算（スケールに合わせて）
  const cogY = rocketSim.centerOfGravity * attitudeDisplayScale;

  // 重心位置に基づいて回転中心を計算
  // 重心位置はノーズ先端からの距離なので、本体高さから重心位置を引くことで底部からの距離を計算
  const cogToBottom = attitudeRocketParams.bodyHeight + attitudeRocketParams.noseHeight - cogY;

  // 姿勢表示用の描画関数 - 新しい引数形式（x, y, params）に対応
  const getBodyPathForAttitude = (x, y, params) => {
    // NaNチェック
    const safeX = isNaN(x) ? 0 : x;
    const safeY = isNaN(y) ? 0 : y;

    // params値のチェック
    const bodyWidth = isNaN(params.bodyWidth) ? 0 : params.bodyWidth;
    const bodyHeight = isNaN(params.bodyHeight) ? 0 : params.bodyHeight;

    const startX = safeX - bodyWidth / 2;
    const endX = safeX + bodyWidth / 2;

    return `M ${startX} ${safeY} 
            L ${startX} ${safeY - bodyHeight} 
            L ${endX} ${safeY - bodyHeight} 
            L ${endX} ${safeY} Z`;
  };

  const getNosePathForAttitude = (x, y, params) => {
    // パラメータの安全な取得
    const noseShape = params.noseShape || rocketSim.noseShape;
    const bodyWidth = getSafeNumber(params.bodyWidth, 30);
    const noseHeight = getSafeNumber(params.noseHeight, 50);

    const startX = x - bodyWidth / 2;
    const endX = x + bodyWidth / 2;
    const baseY = y - params.bodyHeight;
    const topY = baseY - noseHeight;

    if (noseShape === "cone") {
      // 円錐ノーズ
      return `M ${startX} ${baseY} L ${x} ${topY} L ${endX} ${baseY} Z`;
    } else if (noseShape === "parabola") {
      // 放物線ノーズ - 精度向上
      const controlY = topY + noseHeight * 0.15;
      return `M ${startX} ${baseY} 
              C ${startX} ${controlY}, ${x} ${topY}, ${x} ${topY}
              C ${x} ${topY}, ${endX} ${controlY}, ${endX} ${baseY} Z`;
    } else {
      // オジブノーズ - 形状の精度向上
      const halfWidth = bodyWidth / 2;
      return `M ${startX} ${baseY}
              Q ${startX + halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${x} ${topY}
              Q ${endX - halfWidth * 0.1} ${topY + noseHeight * 0.4}, ${endX} ${baseY} Z`;
    }
  };

  // 姿勢表示用に中央のフィンを描画する関数
  const getCenterFinsPathForAttitude = (x, y, params) => {
    const halfThickness = params.finThickness / 2;
    const finBottomY = y;
    const finTopY = finBottomY - params.finBaseWidth;

    // フィンの翼端位置を正確に計算
    const tipTopY = finTopY + params.finSweepLength;
    const tipBottomY = tipTopY + params.finTipWidth;

    // フィンの後端が左右のフィンと揃うように調整
    // フィンの翼端長がボディ後端よりも伸びる場合を計算
    const finExtension = Math.max(0, params.finSweepLength + params.finTipWidth - params.finBaseWidth);

    return `M ${x - halfThickness} ${finTopY}
            L ${x - halfThickness} ${finBottomY}
            L ${x - halfThickness} ${finBottomY + finExtension}
            L ${x + halfThickness} ${finBottomY + finExtension}
            L ${x + halfThickness} ${finBottomY}
            L ${x + halfThickness} ${finTopY} Z`;
  };

  // フィン形状描画関数の改良版
  const getFinPathsForAttitude = (x, y, params) => {
    // 安全にパラメータを取得
    const bodyWidth = getSafeNumber(params.bodyWidth, 30);
    const finHeight = getSafeNumber(params.finHeight, 40);
    const finBaseWidth = getSafeNumber(params.finBaseWidth, 30);
    const finSweepLength = getSafeNumber(params.finSweepLength, 0);
    const finTipWidth = getSafeNumber(params.finTipWidth, 20);
    const finThickness = getSafeNumber(params.finThickness, 2);

    const bodyLeftX = x - bodyWidth / 2;
    const bodyRightX = x + bodyWidth / 2;
    const finBottomY = y;
    const finTopY = finBottomY - finBaseWidth;

    // 後退代がマイナスの場合の処理を追加
    let tipTopY, tipBottomY;

    if (finSweepLength >= 0) {
      // 通常の正の後退代の場合
      tipTopY = finTopY + finSweepLength;
      tipBottomY = tipTopY + finTipWidth;
    } else {
      // 後退代がマイナスの場合（前進翼）
      tipTopY = finTopY - Math.abs(finSweepLength);
      tipBottomY = tipTopY + finTipWidth;
    }

    // フィン形状のパス作成
    const leftFin = `M ${bodyLeftX} ${finTopY}
                   L ${bodyLeftX} ${finBottomY}
                   L ${bodyLeftX - finHeight} ${tipBottomY}
                   L ${bodyLeftX - finHeight} ${tipTopY} Z`;

    const rightFin = `M ${bodyRightX} ${finTopY}
                    L ${bodyRightX} ${finBottomY}
                    L ${bodyRightX + finHeight} ${tipBottomY}
                    L ${bodyRightX + finHeight} ${tipTopY} Z`;

    // 中央フィンの描画 - 後退代がマイナスの場合の処理を追加
    const halfThickness = finThickness / 2;
    let frontExtension = 0;
    let finExtension = 0;

    if (finSweepLength < 0) {
      // 前進翼の場合
      frontExtension = Math.abs(finSweepLength);
    }

    // フィンの後端がボディ後端より後ろに出る場合を計算
    finExtension = Math.max(0, finSweepLength + finTipWidth - finBaseWidth);

    const centerFins = `M ${x - halfThickness} ${finTopY - frontExtension}
                      L ${x - halfThickness} ${finBottomY}
                      L ${x - halfThickness} ${finBottomY + finExtension}
                      L ${x + halfThickness} ${finBottomY + finExtension}
                      L ${x + halfThickness} ${finBottomY}
                      L ${x + halfThickness} ${finTopY - frontExtension} Z`;

    return { leftFin, rightFin, centerFins };
  };

  // 3枚フィン用のフィンパス生成関数
  const getTriFinPathsForAttitude = (x, y, params, deflection = 0) => {
    // 安全にパラメータを取得
    const bodyWidth = getSafeNumber(params.bodyWidth, 30);
    const finHeight = getSafeNumber(params.finHeight, 40);
    const finBaseWidth = getSafeNumber(params.finBaseWidth, 30);
    const finSweepLength = getSafeNumber(params.finSweepLength, 0);
    const finTipWidth = getSafeNumber(params.finTipWidth, 20);
    const finThickness = getSafeNumber(params.finThickness, 2);

    // ボディの中心からの左右のフィン位置のオフセット (sqrt(3)/2 = 1.73/2)
    const offset = (bodyWidth / 2) * 1.73 / 2;
    const leftFinRootX = x - offset;
    const rightFinRootX = x + offset;

    // フィン高さを調整 (1.73/2倍)
    const adjustedFinHeight = finHeight * 1.73 / 2;

    const finBottomY = y;
    const finTopY = finBottomY - finBaseWidth;

    // たわみを考慮した修正
    const deflectionAngle = deflection * Math.PI / 180; // ラジアンに変換

    // 後退代に応じた位置調整
    let adjustedFinTopY, adjustedFinBottomY;

    if (finSweepLength >= 0) {
      // 通常の正の後退代の場合
      adjustedFinTopY = finTopY + finSweepLength;
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    } else {
      // 後退代がマイナスの場合（前進翼）
      adjustedFinTopY = finTopY - Math.abs(finSweepLength);
      adjustedFinBottomY = adjustedFinTopY + finTipWidth;
    }

    // たわみを適用したフィン座標
    const leftFinTipX = leftFinRootX - adjustedFinHeight * Math.cos(deflectionAngle);
    const leftFinTipY = adjustedFinTopY + adjustedFinHeight * Math.sin(deflectionAngle);
    const leftFinBottomTipX = leftFinRootX - adjustedFinHeight * Math.cos(deflectionAngle);
    const leftFinBottomTipY = adjustedFinBottomY + adjustedFinHeight * Math.sin(deflectionAngle);

    const rightFinTipX = rightFinRootX + adjustedFinHeight * Math.cos(deflectionAngle);
    const rightFinTipY = adjustedFinTopY + adjustedFinHeight * Math.sin(deflectionAngle);
    const rightFinBottomTipX = rightFinRootX + adjustedFinHeight * Math.cos(deflectionAngle);
    const rightFinBottomTipY = adjustedFinBottomY + adjustedFinHeight * Math.sin(deflectionAngle);

    const leftFin = `M ${leftFinRootX} ${finTopY}
                  L ${leftFinRootX} ${finBottomY}
                  L ${leftFinBottomTipX} ${leftFinBottomTipY}
                  L ${leftFinTipX} ${leftFinTipY} Z`;

    const rightFin = `M ${rightFinRootX} ${finTopY}
                  L ${rightFinRootX} ${finBottomY}
                  L ${rightFinBottomTipX} ${rightFinBottomTipY}
                  L ${rightFinTipX} ${rightFinTipY} Z`;

    // 中央フィンの描画 - 後退代がマイナスの場合の処理を追加
    const halfThickness = finThickness / 2;
    let frontExtension = 0;
    if (finSweepLength < 0) {
      frontExtension = Math.abs(finSweepLength);
    }

    const centerFins = `M ${x - halfThickness} ${finTopY - frontExtension}
                    L ${x - halfThickness} ${finBottomY}
                    L ${x + halfThickness} ${finBottomY}
                    L ${x + halfThickness} ${finTopY - frontExtension} Z`;

    return { leftFin, rightFin, centerFins };
  };

  // 安全な回転属性を生成する関数
  const getSafeRotationTransform = (rotation, x, y) => {
    // 全ての値にNaNチェック
    const safeRotation = isNaN(rotation) ? 0 : rotation;
    const safeX = isNaN(x) ? SVG_CONFIG.centerX : x;
    const safeY = isNaN(y) ? SVG_CONFIG.groundLevel : y;

    return `rotate(${safeRotation} ${safeX} ${safeY})`;
  };

  // 前回の飛翔結果表示コンポーネント
  const LastFlightResults = ({ results }) => {
    if (!results) return null;

    const {
      maxSpeed, maxHeight, maxDistance,
      isDivergenceOK, isFlutterOK, isDeflectionOK, isAngleStableOK, isAbsoluteAngleOK, isOverallOK,
      launchAngle, windSpeed, windProfile, landing
    } = results;

    return (
      <div className="bg-white border border-gray-200 rounded-lg p-3 mt-4">
        <h4 className="font-semibold mb-2">前回の飛翔結果</h4>

        <div className="grid grid-cols-2 gap-x-2 gap-y-1 text-sm">
          <div>発射角度:</div>
          <div>{launchAngle}°</div>

          <div>風速:</div>
          <div>{windSpeed.toFixed(1)} m/s</div>

          <div>風速プロファイル:</div>
          <div>{WIND_PROFILES[windProfile].name}</div>

          <div>最高高度:</div>
          <div>{maxHeight.toFixed(1)} m</div>

          <div>最高速度:</div>
          <div>{maxSpeed.toFixed(1)} m/s</div>

          <div>最大水平距離:</div>
          <div>{maxDistance.toFixed(1)} m</div>

          {landing && (
            <>
              <div>予測着地距離:</div>
              <div>{landing.landingDistance.toFixed(1)} m</div>

              <div>予測滞空時間:</div>
              <div>{landing.totalFlightTime.toFixed(1)} 秒</div>
            </>
          )}
        </div>

        <div className="mt-2 pt-2 border-t border-gray-300 grid grid-cols-2 gap-2">
          <div className="text-sm">
            <span>フィンダイバージェンス:</span>
            <span className={`ml-1 font-medium ${isDivergenceOK ? 'text-green-600' : 'text-red-600'}`}>
              {isDivergenceOK ? 'OK' : 'NG'}
            </span>
          </div>

          <div className="text-sm">
            <span>フィンフラッター:</span>
            <span className={`ml-1 font-medium ${isFlutterOK ? 'text-green-600' : 'text-red-600'}`}>
              {isFlutterOK ? 'OK' : 'NG'}
            </span>
          </div>

          <div className="text-sm">
            <span>フィンたわみ:</span>
            <span className={`ml-1 font-medium ${isDeflectionOK ? 'text-green-600' : 'text-red-600'}`}>
              {isDeflectionOK ? 'OK' : 'NG'}
            </span>
          </div>

          {/* 姿勢安定性の判定に関する表示を修正 */}
          <div className="text-sm">
            <span>角度変化安定性:</span>
            <span className={`ml-1 font-medium ${isAngleStableOK ? 'text-green-600' : 'text-red-600'}`}>
              {isAngleStableOK ? 'OK' : 'NG'}
            </span>
          </div>

          {/* 絶対角度判定を追加 */}
          <div className="text-sm col-span-2">
            <span>絶対角度安定性:</span>
            <span className={`ml-1 font-medium ${isAbsoluteAngleOK ? 'text-green-600' : 'text-red-600'}`}>
              {isAbsoluteAngleOK ? 'OK' : 'NG'}
            </span>
          </div>

          <div className="mt-2 text-center">
            <span className="font-semibold">総合判定:</span>
            <span className={`ml-2 font-bold ${isOverallOK ? 'text-green-600' : 'text-red-600'}`}>
              {isOverallOK ? 'OK' : 'NG'}
            </span>
          </div>
        </div>
      </div>
    );
  };

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

          {/* 姿勢制御設定セクション - 開発モード時のみ表示 */}
          {devMode && (
            <div className="mt-4">
              <h5 className="font-medium mb-2">姿勢制御設定:</h5>
              <div className="flex flex-col space-y-2">
                <label className="flex items-center">
                  <input
                    type="checkbox"
                    checked={rocketSim.enhancedAttitudeControl}
                    onChange={(e) => rocketSim.setEnhancedAttitudeControl(e.target.checked)}
                    disabled={rocketSim.isLaunched}
                    className="mr-2"
                  />
                  <span>拡張姿勢制御を有効化</span>
                  <span className="ml-2 text-xs text-gray-500">(風見効果など)</span>
                </label>

                <label className={`flex items-center ${!rocketSim.enhancedAttitudeControl ? 'opacity-50' : ''}`}>
                  <input
                    type="checkbox"
                    checked={rocketSim.windAngleLimitation}
                    onChange={(e) => rocketSim.setWindAngleLimitation(e.target.checked)}
                    disabled={rocketSim.isLaunched || !rocketSim.enhancedAttitudeControl}
                    className="mr-2"
                  />
                  <span>風向きによる角度制限 (90°制限)</span>
                </label>
              </div>

              <div className="text-sm text-gray-600 mt-1 pl-5">
                <p>
                  拡張姿勢制御ONで姿勢変化が見られます。
                  角度制限ONで風上時に姿勢角が±90°までに制限されます。
                </p>
              </div>
            </div>
          )}

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
              {/* 着地予測情報を追加 */}
              {rocketSim.landing && (
                <div>予測着地距離: {rocketSim.landing.landingDistance.toFixed(1)} m</div>
              )}
            </div>
            <div className="space-y-2">
              <div>最高到達高度: {rocketSim.currentMaxHeight.toFixed(1)} m</div>
              <div>最高速度: {rocketSim.currentMaxSpeed.toFixed(1)} m/s</div>
              <div>最大水平距離: {rocketSim.currentMaxDistance.toFixed(1)} m</div>
              <div>最大フィンたわみ量: {rocketSim.currentMaxFinDeflection.toFixed(2)} mm</div>
              {/* 着地予測までの時間を追加 */}
              {rocketSim.landing && (
                <div>着地までの時間: {rocketSim.landing.timeToLanding.toFixed(1)} 秒</div>
              )}
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

            {/* 着地予測表示切替ボタンを追加 */}
            {rocketSim.landing && (
              <button
                onClick={() => rocketSim.setShowLandingPrediction(!rocketSim.showLandingPrediction)}
                className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
              >
                {rocketSim.showLandingPrediction ? "着地予測を非表示" : "着地予測を表示"}
              </button>
            )}

            {/* デバッグ表示ボタン - 開発モード時のみ表示 */}
            {devMode && (
              <button
                onClick={() => setDebugView(!debugView)}
                className="px-4 py-2 bg-blue-500 text-white rounded hover:bg-blue-600"
              >
                {debugView ? "通常表示" : "デバッグ表示"}
              </button>
            )}

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

              {/* 着地予測マーカーを追加 */}
              {rocketSim.showLandingPrediction && rocketSim.landing && (
                <g className="landing-prediction-marker">
                  {/* 着地点マーカー */}
                  <circle
                    cx={rocketSim.metersToSvgX(rocketSim.landing.landingX)}
                    cy={550} // 地面の高さ
                    r="5"
                    fill="red"
                  />
                  <circle
                    cx={rocketSim.metersToSvgX(rocketSim.landing.landingX)}
                    cy={550} // 地面の高さ
                    r="10"
                    fill="none"
                    stroke="red"
                    strokeWidth="2"
                    strokeDasharray="2,2"
                  />

                  {/* 着地予測ラベル */}
                  <text
                    x={rocketSim.metersToSvgX(rocketSim.landing.landingX)}
                    y={520} // 地面より少し上
                    textAnchor="middle"
                    fill="red"
                    fontWeight="bold"
                    fontSize="12px"
                  >
                    予測着地点
                  </text>
                  {/* 着地距離表示 - マーカーの少し下に配置 */}
                  <text
                    x={rocketSim.metersToSvgX(rocketSim.landing.landingX)}
                    y={535} // 地面より少し下（グレーエリア内）
                    textAnchor="middle"
                    fill="red"
                    fontWeight="bold"
                    fontSize="12px"
                  >
                    {rocketSim.landing.landingDistance.toFixed(1)}m
                  </text>
                </g>
              )}

            {/* 落下予測線を追加 - 現在位置から着地点までの点線 */}
            {rocketSim.showLandingPrediction && rocketSim.landing && rocketSim.isLaunched && position && position.physicsY > 0 && (
              <path
                d={`M ${rocketSim.metersToSvgX(position.physicsX)} ${rocketSim.metersToSvgY(position.physicsY)} L ${rocketSim.metersToSvgX(rocketSim.landing.landingX)} 550`}
                stroke="red"
                strokeWidth="1.5"
                strokeDasharray="4,4"
                fill="none"
              />
            )}

            {/* ロケットとパラシュートの表示 - パラシュート表示を改良 */}
            {rocketSim.isLaunched && rocketSim.flightData.length > 0 && (
              <>
                {/* パラシュート表示（パラシュート展開時のみ） */}
                {position.isParachuteEjected && (
                  <>
                    <path
                      d={rocketSim.getParachuteStringPaths(
                        rocketSim.metersToSvgX(position.physicsX),
                        rocketSim.metersToSvgY(position.physicsY),
                        position.isParachuteActive,
                        position.parachuteDeploymentProgress,
                        position.rotation
                      )}
                      stroke="#374151"
                      strokeWidth="1"
                      fill="none"
                    />
                    <path
                      d={rocketSim.getParachutePath(
                        rocketSim.metersToSvgX(position.physicsX),
                        rocketSim.metersToSvgY(position.physicsY),
                        position.isParachuteActive,
                        position.parachuteDeploymentProgress,
                        position.rotation
                      )}
                      fill={position.isParachuteActive ? "#FFB300" : "#9ca3af"}
                      stroke="#374151"
                      strokeWidth="2"
                    />
                  </>
                )}

                {/* ロケット */}
                <g transform={getSafeRotationTransform(position.rotation, rocketSim.metersToSvgX(position.physicsX), rocketSim.metersToSvgY(position.physicsY))}>
                  {/* 3枚フィンの場合は先に左右フィンを描画してボディの後ろに表示 */}
                  {rocketSim.finCount === 3 && (
                    <>
                      <path
                        d={getTriFinPathsForAttitude(
                          rocketSim.metersToSvgX(position.physicsX),
                          rocketSim.metersToSvgY(position.physicsY),
                          rocketDisplayParams,
                          position.finDeflection
                        ).leftFin}
                        fill="#6B7280"
                        stroke="#374151"
                      />
                      <path
                        d={getTriFinPathsForAttitude(
                          rocketSim.metersToSvgX(position.physicsX),
                          rocketSim.metersToSvgY(position.physicsY),
                          rocketDisplayParams,
                          position.finDeflection
                        ).rightFin}
                        fill="#6B7280"
                        stroke="#374151"
                      />
                    </>
                  )}

                  {/* ボディ */}
                  <path
                    d={getBodyPathForAttitude(
                      rocketSim.metersToSvgX(position.physicsX),
                      rocketSim.metersToSvgY(position.physicsY),
                      rocketDisplayParams
                    )}
                    fill="#9CA3AF"
                    stroke="#374151"
                  />

                  {/* 中央フィン（両方のフィン設定で共通） */}
                  <path
                    d={rocketSim.finCount === 3
                      ? getTriFinPathsForAttitude(
                        rocketSim.metersToSvgX(position.physicsX),
                        rocketSim.metersToSvgY(position.physicsY),
                        rocketDisplayParams,
                        position.finDeflection
                      ).centerFins
                      : getFinPathsForAttitude(
                        rocketSim.metersToSvgX(position.physicsX),
                        rocketSim.metersToSvgY(position.physicsY),
                        rocketDisplayParams,
                        position.finDeflection
                      ).centerFins
                    }
                    fill="#6B7280"
                    stroke="#374151"
                  />

                  {/* 4枚フィンの場合のみ左右フィン */}
                  {rocketSim.finCount === 4 && (
                    <>
                      <path
                        d={getFinPathsForAttitude(
                          rocketSim.metersToSvgX(position.physicsX),
                          rocketSim.metersToSvgY(position.physicsY),
                          rocketDisplayParams,
                          position.finDeflection
                        ).leftFin}
                        fill="#6B7280"
                        stroke="#374151"
                      />
                      <path
                        d={getFinPathsForAttitude(
                          rocketSim.metersToSvgX(position.physicsX),
                          rocketSim.metersToSvgY(position.physicsY),
                          rocketDisplayParams,
                          position.finDeflection
                        ).rightFin}
                        fill="#6B7280"
                        stroke="#374151"
                      />
                    </>
                  )}

                  {/* ノーズ */}
                  <path
                    d={getNosePathForAttitude(
                      rocketSim.metersToSvgX(position.physicsX),
                      rocketSim.metersToSvgY(position.physicsY),
                      rocketDisplayParams
                    )}
                    fill="#D1D5DB"
                    stroke="#374151"
                  />
                </g>
              </>
            )}

            {/* 未発射時のロケット表示 */}
            {!rocketSim.isLaunched && (
              <g transform={getSafeRotationTransform(rocketSim.launchAngle, 400, 550)}>
                {/* 3枚フィンの場合は先に左右フィンを描画してボディの後ろに表示 */}
                {rocketSim.finCount === 3 && (
                  <>
                    <path
                      d={getTriFinPathsForAttitude(
                        400,
                        550,
                        rocketDisplayParams
                      ).leftFin}
                      fill="#6B7280"
                      stroke="#374151"
                    />
                    <path
                      d={getTriFinPathsForAttitude(
                        400,
                        550,
                        rocketDisplayParams
                      ).rightFin}
                      fill="#6B7280"
                      stroke="#374151"
                    />
                  </>
                )}

                {/* ボディ */}
                <path
                  d={getBodyPathForAttitude(400, 550, rocketDisplayParams)}
                  fill="#9CA3AF"
                  stroke="#374151"
                />

                {/* 中央フィン（両方のフィン設定で共通） */}
                <path
                  d={rocketSim.finCount === 3
                    ? getTriFinPathsForAttitude(400, 550, rocketDisplayParams).centerFins
                    : getFinPathsForAttitude(400, 550, rocketDisplayParams).centerFins
                  }
                  fill="#6B7280"
                  stroke="#374151"
                />

                {/* 4枚フィンの場合のみ左右フィン */}
                {rocketSim.finCount === 4 && (
                  <>
                    <path
                      d={getFinPathsForAttitude(400, 550, rocketDisplayParams).leftFin}
                      fill="#6B7280"
                      stroke="#374151"
                    />
                    <path
                      d={getFinPathsForAttitude(400, 550, rocketDisplayParams).rightFin}
                      fill="#6B7280"
                      stroke="#374151"
                    />
                  </>
                )}

                {/* ノーズ */}
                <path
                  d={getNosePathForAttitude(400, 550, rocketDisplayParams)}
                  fill="#D1D5DB"
                  stroke="#374151"
                />
              </g>
            )}

            {/* デバッグ情報表示 */}
            {debugView && (
              <g>
                <rect
                  x={10}
                  y={10}
                  width={280}
                  height={380}
                  fill="rgba(255,255,255,0.8)"
                  stroke="#666"
                  strokeWidth="1"
                />
                <text x={20} y={30} fontSize="12" fill="#333">
                  時間: {rocketSim.currentTime.toFixed(2)} 秒
                </text>
                <text x={20} y={50} fontSize="12" fill="#333">
                  高度: {rocketSim.currentHeight.toFixed(2)} m
                </text>
                <text x={20} y={70} fontSize="12" fill="#333">
                  速度: {rocketSim.currentSpeed.toFixed(2)} m/s
                </text>
                <text x={20} y={90} fontSize="12" fill="#333">
                  角度: {(position.rotation % 360).toFixed(2)}°
                </text>
                <text x={20} y={110} fontSize="12" fill="#333">
                  現在水平距離: {rocketSim.currentDistance.toFixed(2)} m
                </text>
                <text x={20} y={130} fontSize="12" fill="#333">
                  フェーズ: {rocketSim.getCurrentFlightPhase()}
                </text>
                <text x={20} y={150} fontSize="12" fill="#333">
                  パラシュート展開率: {position.parachuteDeploymentProgress.toFixed(2)}
                </text>
                <text x={20} y={170} fontSize="12" fill="#333">
                  アニメーション速度: 0.2倍速
                </text>
                <text x={20} y={190} fontSize="12" fill="#333">
                  推力終了時高度: {rocketSim.keyPoints.thrustEnd?.height?.toFixed(2) || "N/A"} m
                </text>
                <text x={20} y={210} fontSize="12" fill="#333">
                  横風速度: {rocketSim.windSpeed.toFixed(2)} m/s
                </text>
                {position.effectiveWindSpeed !== undefined && (
                  <text x={20} y={230} fontSize="12" fill="#333">
                    実効風速: {position.effectiveWindSpeed.toFixed(2)} m/s
                  </text>
                )}
                <text x={20} y={250} fontSize="12" fill="#333">
                  最大水平距離: {rocketSim.currentMaxDistance.toFixed(2)} m
                </text>
                <text x={20} y={270} fontSize="12" fill="#333">
                  表示スケール: {rocketSim.trajectoryScale.toFixed(2)} px/m
                </text>
                <text x={20} y={290} fontSize="12" fill="#333">
                  ロケットスケール: {rocketSim.rocketScale.toFixed(5)}
                </text>
                <text x={20} y={310} fontSize="12" fill="#333">
                  フィンたわみ量: {position.finDeflection.toFixed(2)} mm
                </text>
                {position.torque !== undefined && (
                  <text x={20} y={330} fontSize="12" fill="#333">
                    トルク: {position.torque.toFixed(6)}
                  </text>
                )}
                <text x={20} y={350} fontSize="12" fill="#333">
                  風向き: {rocketSim.windSpeed > 0 ? "左→右" : rocketSim.windSpeed < 0 ? "右→左" : "無風"}
                </text>
                <text x={20} y={370} fontSize="12" fill="#333">
                  角度制限: {rocketSim.windSpeed !== 0 ? `${-Math.sign(rocketSim.windSpeed) * 90}°` : "なし"}
                </text>
              </g>
            )}

            {/* 着地予測情報をデバッグ表示に追加 */}
            {rocketSim.landing && (
              <>
                <text x={20} y={390} fontSize="12" fill="#333">
                  予測着地距離: {rocketSim.landing.landingDistance.toFixed(2)} m
                </text>
                <text x={20} y={410} fontSize="12" fill="#333">
                予測滞空時間: {rocketSim.landing.totalFlightTime.toFixed(1)} 秒
                </text>
              </>
            )}

          </svg>
        </div>

        {/* 右側: 円形の姿勢表示窓 - 1つだけに修正 */}
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
                {/* 重心を中心に配置するための調整 */}
                <g transform={`translate(0, 0)`}>
                  {/* ロケット本体 - 重心が中心に来るように配置調整 */}
                  <g transform={`translate(0, ${scaledRocketNoseHeight + scaledRocketBodyHeight - scaledCogY})`}>
                    {/* 3枚フィンの場合は先に左右フィンを描画してボディの後ろに表示 */}
                    {rocketSim.finCount === 3 && (
                      <>
                        {/* 左フィン - 位置と幅を調整 */}
                        <path
                          d={(() => {
                            const offset = (attitudeRocketParams.bodyWidth / 2) * 1.73 / 2;
                            const height = attitudeRocketParams.finHeight * 1.73 / 2;
                            const leftFinRootX = -offset;
                            const finTopY = -attitudeRocketParams.finBaseWidth;
                            const finBottomY = 0;

                            // 後退代に応じた位置調整
                            let adjustedFinTopY, adjustedFinBottomY;
                            if (attitudeRocketParams.finSweepLength >= 0) {
                              adjustedFinTopY = finTopY + attitudeRocketParams.finSweepLength;
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            } else {
                              adjustedFinTopY = finTopY - Math.abs(attitudeRocketParams.finSweepLength);
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            }

                            return `M ${leftFinRootX} ${finTopY}
                    L ${leftFinRootX} ${finBottomY}
                    L ${leftFinRootX - height} ${adjustedFinBottomY}
                    L ${leftFinRootX - height} ${adjustedFinTopY} Z`;
                          })()}
                          fill="#6B7280"
                          stroke="#374151"
                          strokeWidth="0.5"
                        />

                        {/* 右フィン - 位置と幅を調整 */}
                        <path
                          d={(() => {
                            const offset = (attitudeRocketParams.bodyWidth / 2) * 1.73 / 2;
                            const height = attitudeRocketParams.finHeight * 1.73 / 2;
                            const rightFinRootX = offset;
                            const finTopY = -attitudeRocketParams.finBaseWidth;
                            const finBottomY = 0;

                            // 後退代に応じた位置調整
                            let adjustedFinTopY, adjustedFinBottomY;
                            if (attitudeRocketParams.finSweepLength >= 0) {
                              adjustedFinTopY = finTopY + attitudeRocketParams.finSweepLength;
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            } else {
                              adjustedFinTopY = finTopY - Math.abs(attitudeRocketParams.finSweepLength);
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            }

                            return `M ${rightFinRootX} ${finTopY}
                    L ${rightFinRootX} ${finBottomY}
                    L ${rightFinRootX + height} ${adjustedFinBottomY}
                    L ${rightFinRootX + height} ${adjustedFinTopY} Z`;
                          })()}
                          fill="#6B7280"
                          stroke="#374151"
                          strokeWidth="0.5"
                        />
                      </>
                    )}

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

                    {/* ノーズ */}
                    <path
                      d={getNosePathForAttitude(0, 0, {
                        ...attitudeRocketParams,
                        bodyHeight: attitudeRocketParams.bodyHeight,
                        noseShape: rocketSim.noseShape
                      })}
                      fill="#D1D5DB"
                      stroke="#374151"
                      strokeWidth="0.5"
                    />

                    {/* 4枚フィンの場合のみ左右フィン */}
                    {rocketSim.finCount === 4 && (
                      <>
                        <path
                          d={(() => {
                            const leftFinRootX = -attitudeRocketParams.bodyWidth / 2;
                            const finTopY = -attitudeRocketParams.finBaseWidth;
                            const finBottomY = 0;

                            // 後退代に応じた位置調整
                            let adjustedFinTopY, adjustedFinBottomY;
                            if (attitudeRocketParams.finSweepLength >= 0) {
                              adjustedFinTopY = finTopY + attitudeRocketParams.finSweepLength;
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            } else {
                              adjustedFinTopY = finTopY - Math.abs(attitudeRocketParams.finSweepLength);
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            }

                            return `M ${leftFinRootX} ${finTopY}
                    L ${leftFinRootX} ${finBottomY}
                    L ${leftFinRootX - attitudeRocketParams.finHeight} ${adjustedFinBottomY}
                    L ${leftFinRootX - attitudeRocketParams.finHeight} ${adjustedFinTopY} Z`;
                          })()}
                          fill="#6B7280"
                          stroke="#374151"
                          strokeWidth="0.5"
                        />

                        <path
                          d={(() => {
                            const rightFinRootX = attitudeRocketParams.bodyWidth / 2;
                            const finTopY = -attitudeRocketParams.finBaseWidth;
                            const finBottomY = 0;

                            // 後退代に応じた位置調整
                            let adjustedFinTopY, adjustedFinBottomY;
                            if (attitudeRocketParams.finSweepLength >= 0) {
                              adjustedFinTopY = finTopY + attitudeRocketParams.finSweepLength;
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            } else {
                              adjustedFinTopY = finTopY - Math.abs(attitudeRocketParams.finSweepLength);
                              adjustedFinBottomY = adjustedFinTopY + attitudeRocketParams.finTipWidth;
                            }

                            return `M ${rightFinRootX} ${finTopY}
                    L ${rightFinRootX} ${finBottomY}
                    L ${rightFinRootX + attitudeRocketParams.finHeight} ${adjustedFinBottomY}
                    L ${rightFinRootX + attitudeRocketParams.finHeight} ${adjustedFinTopY} Z`;
                          })()}
                          fill="#6B7280"
                          stroke="#374151"
                          strokeWidth="0.5"
                        />
                      </>
                    )}

                    {/* センターフィン - 共通で表示 */}
                    {(() => {
                      const halfThickness = attitudeRocketParams.finThickness / 2;
                      const finTopY = -attitudeRocketParams.finBaseWidth;
                      const finBottomY = 0;

                      // 後退代がマイナスの場合の処理
                      let frontExtension = 0;
                      let finExtension = 0;

                      if (attitudeRocketParams.finSweepLength < 0) {
                        // 前進翼の場合
                        frontExtension = Math.abs(attitudeRocketParams.finSweepLength);
                      }

                      // フィンの後端がボディ後端より後ろに出る場合を計算
                      finExtension = Math.max(0, attitudeRocketParams.finSweepLength +
                        attitudeRocketParams.finTipWidth - attitudeRocketParams.finBaseWidth);

                      return (
                        <path
                          d={`M ${-halfThickness} ${finTopY - frontExtension}
              L ${-halfThickness} ${finBottomY}
              L ${-halfThickness} ${finBottomY + finExtension}
              L ${halfThickness} ${finBottomY + finExtension}
              L ${halfThickness} ${finBottomY}
              L ${halfThickness} ${finTopY - frontExtension} Z`}
                          fill="#6B7280"
                          stroke="#374151"
                          strokeWidth="0.5"
                        />
                      );
                    })()}
                  </g>

                  {/* 重心位置のマーカー - 回転中心 */}
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

      {/* 結果ポップアップ */ }
  {
    rocketSim.showResultsPopup && (
      <ResultsPopup
        results={rocketSim.flightResults}
        onClose={rocketSim.handleCloseResultsPopup}
      />
    )
  }
    </div >
  );
};