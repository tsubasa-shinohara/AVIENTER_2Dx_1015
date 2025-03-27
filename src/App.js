import React from 'react';
import { BrowserRouter as Router, Routes, Route, Navigate } from 'react-router-dom';
import Login from './components/rocket/Login';
import IntegratedRocketSimulator from './components/rocket/RocketSimulator';

const App = () => {
  // ログイン状態を確認する関数（localStorageを使用）
  const isLoggedIn = () => {
    return localStorage.getItem('isLoggedIn') === 'true';
  };

  return (
    <Router>
      <Routes>
        {/* ログイン画面 */}
        <Route path="/login" element={<Login />} />
        
        {/* ロケットシミュレーター */}
        <Route
          path="/"
          element={
            isLoggedIn() ? (
              <IntegratedRocketSimulator />
            ) : (
              <Navigate to="/login" replace />
            )
          }
        />
      </Routes>
    </Router>
  );
};

export default App;