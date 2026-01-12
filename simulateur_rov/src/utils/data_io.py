"""Import/Export de données"""
import numpy as np
import pandas as pd
import h5py
import os


def save_results(system, solution, filepath, format='h5'):
    """
    Sauvegarde les résultats de simulation
    
    Parameters:
    -----------
    system : ROVSystem
        Système ROV
    solution : scipy.integrate.OdeResult
        Solution de l'intégration
    filepath : str
        Chemin de sauvegarde
    format : str
        Format de sauvegarde ('h5', 'csv', 'npz')
    """
    os.makedirs(os.path.dirname(filepath) if os.path.dirname(filepath) else '.', exist_ok=True)
    
    if format == 'h5':
        save_h5(system, solution, filepath)
    elif format == 'csv':
        save_csv(system, solution, filepath)
    elif format == 'npz':
        save_npz(system, solution, filepath)
    else:
        raise ValueError(f"Format non supporté: {format}")


def save_h5(system, solution, filepath):
    """Sauvegarde en format HDF5"""
    with h5py.File(filepath, 'w') as f:
        # Temps
        f.create_dataset('time', data=solution.t)
        
        # État complet
        f.create_dataset('state', data=solution.y.T)
        
        # Extraire les variables principales
        N = system.N
        n_points = len(solution.t)
        
        x_rov = np.zeros(n_points)
        y_rov = np.zeros(n_points)
        vx_rov = np.zeros(n_points)
        vy_rov = np.zeros(n_points)
        x_boat = np.zeros(n_points)
        vx_boat = np.zeros(n_points)
        L = np.zeros(n_points)
        
        for i, t in enumerate(solution.t):
            y = solution.sol(t)
            (x_rov[i], y_rov[i], vx_rov[i], vy_rov[i],
             x_boat[i], vx_boat[i], _, _, _, L[i]) = system.unpack_state(y)
        
        f.create_dataset('x_rov', data=x_rov)
        f.create_dataset('y_rov', data=y_rov)
        f.create_dataset('vx_rov', data=vx_rov)
        f.create_dataset('vy_rov', data=vy_rov)
        f.create_dataset('x_boat', data=x_boat)
        f.create_dataset('vx_boat', data=vx_boat)
        f.create_dataset('L', data=L)
        
        # Métadonnées
        f.attrs['N_segments'] = system.N


def save_csv(system, solution, filepath):
    """Sauvegarde en format CSV"""
    N = system.N
    n_points = len(solution.t)
    
    # Extraire les variables principales
    data = {
        'time': solution.t,
        'x_rov': np.zeros(n_points),
        'y_rov': np.zeros(n_points),
        'vx_rov': np.zeros(n_points),
        'vy_rov': np.zeros(n_points),
        'x_boat': np.zeros(n_points),
        'vx_boat': np.zeros(n_points),
        'L': np.zeros(n_points)
    }
    
    for i, t in enumerate(solution.t):
        y = solution.sol(t)
        (data['x_rov'][i], data['y_rov'][i],
         data['vx_rov'][i], data['vy_rov'][i],
         data['x_boat'][i], data['vx_boat'][i],
         _, _, _, data['L'][i]) = system.unpack_state(y)
    
    df = pd.DataFrame(data)
    df.to_csv(filepath, index=False)


def save_npz(system, solution, filepath):
    """Sauvegarde en format NPZ (NumPy)"""
    np.savez(filepath,
             time=solution.t,
             state=solution.y)


def load_results(filepath):
    """
    Charge les résultats d'une simulation
    
    Parameters:
    -----------
    filepath : str
        Chemin vers le fichier
    
    Returns:
    --------
    dict
        Données chargées
    """
    ext = os.path.splitext(filepath)[1]
    
    if ext == '.h5':
        return load_h5(filepath)
    elif ext == '.csv':
        return load_csv(filepath)
    elif ext == '.npz':
        return load_npz(filepath)
    else:
        raise ValueError(f"Format non supporté: {ext}")


def load_h5(filepath):
    """Charge depuis HDF5"""
    data = {}
    with h5py.File(filepath, 'r') as f:
        for key in f.keys():
            data[key] = f[key][:]
        # Métadonnées
        if 'N_segments' in f.attrs:
            data['N_segments'] = f.attrs['N_segments']
    return data


def load_csv(filepath):
    """Charge depuis CSV"""
    df = pd.read_csv(filepath)
    return df.to_dict('list')


def load_npz(filepath):
    """Charge depuis NPZ"""
    data = np.load(filepath)
    return {key: data[key] for key in data.files}

