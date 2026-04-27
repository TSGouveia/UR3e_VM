// Lê a variável que passamos no script .sh ou usa localhost por defeito
export const API_BASE = import.meta.env.VITE_API_BASE_URL || "http://localhost:8000";
