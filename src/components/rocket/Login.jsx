import React, { useState } from 'react';
import { useNavigate } from 'react-router-dom';
import { useTranslation } from 'react-i18next';
import LanguageSwitcher from '../LanguageSwitcher';

const Login = () => {
  const { t } = useTranslation();
  const VALID_ID = 'avienter2025';
  const VALID_PASSWORD = 'launchsuccess';
  const [id, setId] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const navigate = useNavigate();

  const handleLogin = (e) => {
    e.preventDefault();
    if (id === VALID_ID && password === VALID_PASSWORD) {
      localStorage.setItem('isLoggedIn', true); // ログイン状態を保存
      navigate('/'); // ロケットシミュレーター画面に遷移
    } else {
      setError(t('login.error'));
    }
  };

  return (
    <div style={styles.container}>
      <div style={styles.card}>
        <div style={{ position: 'absolute', top: '20px', right: '20px' }}>
          <LanguageSwitcher />
        </div>
        <h2 style={styles.title}>{t('login.title')}</h2>
        <form onSubmit={handleLogin} style={styles.form}>
          <div style={styles.inputGroup}>
            <label style={styles.label}>{t('login.id')}</label>
            <input
              type="text"
              placeholder={t('login.idPlaceholder')}
              value={id}
              onChange={(e) => setId(e.target.value)}
              style={styles.input}
            />
          </div>
          <div style={styles.inputGroup}>
            <label style={styles.label}>{t('login.password')}</label>
            <input
              type="password"
              placeholder={t('login.passwordPlaceholder')}
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              style={styles.input}
            />
          </div>
          {error && <p style={styles.error}>{error}</p>}
          <button type="submit" style={styles.button}>{t('login.loginButton')}</button>
        </form>
      </div>
    </div>
  );
};

const styles = {
  container: {
    display: 'flex',
    justifyContent: 'center',
    alignItems: 'center',
    height: '100vh',
    backgroundColor: '#f0f2f5',
    position: 'relative',
  },
  card: {
    width: '100%',
    maxWidth: '400px',
    padding: '30px',
    backgroundColor: '#fff',
    borderRadius: '10px',
    boxShadow: '0 4px 8px rgba(0, 0, 0, 0.1)',
    textAlign: 'center',
    position: 'relative',
  },
  title: {
    fontSize: '24px',
    fontWeight: 'bold',
    marginBottom: '20px',
    color: '#333',
  },
  form: {
    display: 'flex',
    flexDirection: 'column',
  },
  inputGroup: {
    marginBottom: '15px',
    textAlign: 'left',
  },
  label: {
    fontSize: '14px',
    fontWeight: 'bold',
    marginBottom: '5px',
    display: 'block',
    color: '#555',
  },
  input: {
    width: '100%',
    padding: '10px',
    fontSize: '14px',
    borderRadius: '5px',
    border: '1px solid #ccc',
    outline: 'none',
    transition: 'border-color 0.3s',
  },
  inputFocus: {
    borderColor: '#007BFF',
  },
  button: {
    padding: '10px 20px',
    fontSize: '16px',
    fontWeight: 'bold',
    color: '#fff',
    backgroundColor: '#007BFF',
    border: 'none',
    borderRadius: '5px',
    cursor: 'pointer',
    transition: 'background-color 0.3s',
  },
  buttonHover: {
    backgroundColor: '#0056b3',
  },
  error: {
    color: 'red',
    fontSize: '14px',
    marginTop: '10px',
  },
};

export default Login;
