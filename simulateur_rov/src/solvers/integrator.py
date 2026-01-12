"""Intégrateur temporel"""
from scipy.integrate import solve_ivp
import numpy as np


class TimeIntegrator:
    """Intégrateur pour résoudre les équations différentielles"""
    
    def __init__(self, method='RK45', rtol=1e-6, atol=1e-8, max_step=0.1):
        """
        Initialise l'intégrateur
        
        Parameters:
        -----------
        method : str
            Méthode d'intégration ('RK45', 'DOP853', 'BDF', etc.)
        rtol : float
            Tolérance relative
        atol : float
            Tolérance absolue
        max_step : float
            Pas de temps maximum (s)
        """
        self.method = method
        self.rtol = rtol
        self.atol = atol
        self.max_step = max_step
    
    def integrate(self, fun, t_span, y0, dense_output=True):
        """
        Intègre le système d'équations différentielles
        
        Parameters:
        -----------
        fun : callable
            Fonction f(t, y) retournant dy/dt
        t_span : tuple
            (t0, tf) intervalle de temps
        y0 : array
            Conditions initiales
        dense_output : bool
            Si True, permet l'interpolation entre les pas de temps
        
        Returns:
        --------
        solution
            Objet solution de scipy.integrate.solve_ivp
        """
        sol = solve_ivp(
            fun=fun,
            t_span=t_span,
            y0=y0,
            method=self.method,
            rtol=self.rtol,
            atol=self.atol,
            max_step=self.max_step,
            dense_output=dense_output
        )
        
        return sol

