package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.awt.Point;
import java.util.*;

/**
 * Clase que implementa una heurística basada en la distancia mínima al objetivo
 * utilizando una variante optimizada del algoritmo de Dijkstra adaptado al juego Hex.
 * Además, incorpora métricas estratégicas adicionales como movilidad, conectividad y
 * flexibilidad de rutas para una evaluación más completa del estado del juego.
 */
public class HexHeuristica {
    
    // Pesos para las diferentes métricas de la heurística
    public static int PESO_MOVILIDAD = 1;
    public static int PESO_CONECTIVIDAD = 4;
    public static int PESO_FLEXIBILIDAD = 2;
    public static int PESO_PUENTES_VIRTUALES = 5;
    public static int PESO_GRUPOS_AISLADOS = 10; // Nuevo peso para grupos aislados
    
    /**
     * Evalúa el estado actual del juego y devuelve un valor heurístico.
     *
     * @param gameState  Estado actual del juego.
     * @param profundidad Profundidad actual en el árbol de búsqueda.
     * @return Valor de la evaluación heurística.
     */
    public int evaluarEstado(HexGameStatus gameState, int profundidad) {
        if (gameState.isGameOver()) {
            return (gameState.GetWinner() == gameState.getCurrentPlayer()) ? Integer.MAX_VALUE / 2 : -Integer.MAX_VALUE / 2;
        }

        // Evaluación rápida para niveles profundos
        if (profundidad > 3) {
            return evaluarEstadoRapido(gameState);
        }

        // Evaluación detallada para niveles superiores
        return evaluarEstadoDetallado(gameState);
    }

    /**
     * Realiza una evaluación simplificada basada en distancias mínimas al objetivo.
     *
     * @param gameState Estado actual del juego.
     * @return Valor de la evaluación rápida.
     */
    private int evaluarEstadoRapido(HexGameStatus gameState) {
        PlayerType currentPlayer = gameState.getCurrentPlayer();
        PlayerType opponent = (currentPlayer == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;

        // Distancias mínimas
        int distCurrent = calcularDistanciaMinima(gameState, currentPlayer);
        int distOpponent = calcularDistanciaMinima(gameState, opponent);

        // Heurística básica: Diferencia de distancias
        return distOpponent - distCurrent;
    }

    /**
     * Realiza una evaluación detallada del estado del juego, incorporando la penalización
     * por grupos aislados.
     *
     * @param gameState Estado actual del juego.
     * @return Valor de la evaluación detallada.
     */
    public int evaluarEstadoDetallado(HexGameStatus gameState) {
        if (gameState.isGameOver()) {
            // Asignar valor extremo según el ganador.
            return (gameState.GetWinner() == gameState.getCurrentPlayer()) ? Integer.MAX_VALUE / 2 : -Integer.MAX_VALUE / 2;
        }

        PlayerType currentPlayer = gameState.getCurrentPlayer();
        PlayerType opponent = (currentPlayer == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
        ArrayList<Point> misFichas = obtenerFichasPropias(gameState, currentPlayer);
        ArrayList<Point> fichasOponente = obtenerFichasPropias(gameState, opponent);
        int size = gameState.getSize();
        int score = 0;

        // Calcular distancia mínima para el jugador actual usando Dijkstra optimizado.
        int distCurrent = calcularDistanciaMinima(gameState, currentPlayer);
        if (distCurrent == 0) return Integer.MAX_VALUE / 2; // Estado de victoria.

        // Calcular distancia mínima para el oponente usando Dijkstra optimizado.
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if (distOpponent == 0) return -Integer.MAX_VALUE / 2; // Estado de derrota.

        // Heurística principal: diferencia de distancias.
        score += distOpponent - distCurrent;

        // Bonificación por posición central.
        Point center = new Point(size / 2, size / 2);
        for (Point ficha : misFichas) {
            if (ficha.equals(center)) {
                score += 5; // Bonificación por estar en el centro.
            }
        }

        // Evaluar conexiones virtuales para el oponente y el jugador actual.
        for (Point ficha : misFichas) {
            score += evaluarConexionesVirtuales(gameState, ficha, opponent) - evaluarConexionesVirtuales(gameState, ficha, currentPlayer);
        }

        // Penalización por la cercanía del oponente a la victoria.
        score += bloquearCaminoDelOponente(gameState, opponent);

        // Añadir métricas estratégicas adicionales.
        // 1. Movilidad
        int movilidad = calcularMovilidad(gameState, currentPlayer) - calcularMovilidad(gameState, opponent);
        score += movilidad * PESO_MOVILIDAD;

        // 2. Conectividad
        int conectividad = evaluarConectividad(gameState, currentPlayer) - evaluarConectividad(gameState, opponent);
        score += conectividad * PESO_CONECTIVIDAD;

        // 3. Flexibilidad de Rutas
        int flexibilidad = evaluarFlexibilidadRutas(gameState, currentPlayer) - evaluarFlexibilidadRutas(gameState, opponent);
        score += flexibilidad * PESO_FLEXIBILIDAD;

        // 4. Puentes Virtuales
        int puentesVirtuales = evaluarPuentesVirtuales(gameState, currentPlayer) - evaluarPuentesVirtuales(gameState, opponent);
        score += puentesVirtuales * PESO_PUENTES_VIRTUALES;

        // 5. Penalización por Grupos Aislados
        int penalizacionAislados = penalizarGruposAislados(gameState, currentPlayer);
        score -= penalizacionAislados * PESO_GRUPOS_AISLADOS;

        // Penalizar grupos aislados del oponente (opcional)
        int penalizacionAisladosOponente = penalizarGruposAislados(gameState, opponent);
        score += penalizacionAisladosOponente * PESO_GRUPOS_AISLADOS;

        return score;
    }

    /**
     * Identifica y penaliza los grupos de fichas aislados del jugador.
     *
     * @param gameState     Estado actual del juego.
     * @param currentPlayer Jugador actual.
     * @return Número de grupos aislados encontrados.
     */
    private int penalizarGruposAislados(HexGameStatus gameState, PlayerType currentPlayer) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, currentPlayer);
        int size = gameState.getSize();
        UnionFind uf = new UnionFind(size * size);

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Unir fichas adyacentes.
        for (Point ficha : fichas) {
            int index1 = ficha.x * size + ficha.y;
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == playerToId(currentPlayer)) {
                    int index2 = neighbor.x * size + neighbor.y;
                    uf.union(index1, index2);
                }
            }
        }

        // Agrupar fichas por sus raíces en Union-Find.
        Map<Integer, List<Point>> grupos = new HashMap<>();
        for (Point ficha : fichas) {
            int index = ficha.x * size + ficha.y;
            int root = uf.find(index);
            grupos.computeIfAbsent(root, k -> new ArrayList<>()).add(ficha);
        }

        // Determinar cuáles grupos están conectados a las fronteras del tablero.
        Set<Integer> gruposConectados = new HashSet<>();
        for (Map.Entry<Integer, List<Point>> entry : grupos.entrySet()) {
            boolean conectadoSuperior = false;
            boolean conectadoInferior = false;
            boolean conectadoIzquierda = false;
            boolean conectadoDerecha = false;

            for (Point ficha : entry.getValue()) {
                if (ficha.y == 0) conectadoIzquierda = true;
                if (ficha.y == size - 1) conectadoDerecha = true;
                if (ficha.x == 0) conectadoSuperior = true;
                if (ficha.x == size - 1) conectadoInferior = true;
            }

            // Para PLAYER1, conectarse de superior a inferior.
            // Para PLAYER2, conectarse de izquierda a derecha.
            if (currentPlayer == PlayerType.PLAYER1) {
                if (conectadoSuperior && conectadoInferior) {
                    gruposConectados.add(entry.getKey());
                }
            } else { // PLAYER2
                if (conectadoIzquierda && conectadoDerecha) {
                    gruposConectados.add(entry.getKey());
                }
            }
        }

        // Cualquier grupo que no esté en gruposConectados se considera aislado.
        int gruposAislados = 0;
        for (Map.Entry<Integer, List<Point>> entry : grupos.entrySet()) {
            if (!gruposConectados.contains(entry.getKey())) {
                gruposAislados++;
            }
        }

        return gruposAislados;
    }

    /**
     * Crea una clave única para un puente virtual basado en las posiciones de las fichas.
     *
     * @param ficha1 Primera ficha del puente.
     * @param ficha2 Segunda ficha del puente.
     * @return Cadena única representando el puente.
     */
    private String crearClavePuente(Point ficha1, Point ficha2) {
        // Ordenar las fichas para asegurar la unicidad de la clave.
        if (ficha1.x < ficha2.x || (ficha1.x == ficha2.x && ficha1.y <= ficha2.y)) {
            return ficha1.x + "," + ficha1.y + "-" + ficha2.x + "," + ficha2.y;
        } else {
            return ficha2.x + "," + ficha2.y + "-" + ficha1.x + "," + ficha1.y;
        }
    }

    /**
     * Penaliza el estado si el oponente tiene una ruta corta hacia la victoria.
     *
     * @param gameState Estado actual del juego.
     * @param opponent  Jugador oponente.
     * @return Penalización basada en la proximidad del oponente a la victoria.
     */
    private int bloquearCaminoDelOponente(HexGameStatus gameState, PlayerType opponent) {
        int distOpponent = calcularDistanciaMinima(gameState, opponent);
        if (distOpponent < 999999) {
            // Penalización inversamente proporcional a la distancia del oponente.
            // Cuanto menor sea la distancia, mayor la penalización.
            return -(1000 / (distOpponent + 1)); // +1 para evitar división por cero.
        }
        return 0;
    }

    /**
     * Calcula la movilidad de un jugador, es decir, la cantidad de celdas vacías adyacentes
     * a las fichas del jugador. Una mayor movilidad indica una mayor flexibilidad en las
     * opciones de movimiento.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se calcula la movilidad.
     * @return Número total de celdas vacías adyacentes a las fichas del jugador.
     */
    private int calcularMovilidad(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int movilidad = 0;
        int size = gameState.getSize();

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Utilizar un conjunto para evitar contar la misma celda vacía múltiples veces.
        Set<Point> celdasVacias = new HashSet<>();
        for (Point ficha : fichas) {
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == 0) {
                    celdasVacias.add(neighbor);
                }
            }
        }

        movilidad = celdasVacias.size();
        return movilidad;
    }

    /**
     * Evalúa la conectividad de un jugador, es decir, cuántas agrupaciones conectadas
     * de fichas tiene el jugador. Menos agrupaciones indican una mejor conectividad.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se evalúa la conectividad.
     * @return Puntuación basada en la conectividad. Menos agrupaciones resultan en una mayor puntuación.
     */
    private int evaluarConectividad(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int size = gameState.getSize();
        UnionFind uf = new UnionFind(size * size);

        // Definir direcciones posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Unir fichas adyacentes.
        for (Point ficha : fichas) {
            int index1 = ficha.x * size + ficha.y;
            for (Point d : directions) {
                Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
                if (estaDentro(neighbor, size) && gameState.getPos(neighbor.x, neighbor.y) == playerToId(player)) {
                    int index2 = neighbor.x * size + neighbor.y;
                    uf.union(index1, index2);
                }
            }
        }

        // Contar el número de grupos conectados.
        Set<Integer> grupos = new HashSet<>();
        for (Point ficha : fichas) {
            int index = ficha.x * size + ficha.y;
            grupos.add(uf.find(index));
        }

        // Menos grupos indican mejor conectividad.
        // Convertimos el número de grupos en una puntuación donde menos grupos son mejores.
        int numeroGrupos = grupos.size();
        if (numeroGrupos == 1) {
            return 30; // Excelente conectividad.
        } else if (numeroGrupos <= 3) {
            return 20; // Buena conectividad.
        } else if (numeroGrupos <= 5) {
            return 10; // Conectividad aceptable.
        } else {
            return -10; // Pobre conectividad.
        }
    }

    /**
     * Evalúa la flexibilidad de rutas de un jugador, es decir, cuántas rutas alternativas
     * tiene el jugador hacia la victoria. Más rutas indican mayor resiliencia ante bloqueos.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se evalúa la flexibilidad de rutas.
     * @return Puntuación basada en la flexibilidad de rutas.
     */
    private int evaluarFlexibilidadRutas(HexGameStatus gameState, PlayerType player) {
        int size = gameState.getSize();
        int playerId = playerToId(player);
        int flexibilidad = 0;

        // Definir movimientos posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        // Recuperar celdas de inicio y fin según el jugador.
        List<Point> fuentes = (player == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);
        List<Point> destinos = (player == PlayerType.PLAYER1) ? getTransitableBottom(gameState, playerId) : getTransitableRight(gameState, playerId);

        // Realizar búsquedas de rutas y contar celdas de destino alcanzables.
        Set<Point> alcanzables = new HashSet<>();

        for (Point fuente : fuentes) {
            alcanzables.addAll(bfsAlcanzables(gameState, fuente, playerId, destinos));
        }

        flexibilidad = alcanzables.size();

        // Convertir el número de celdas alcanzables en una puntuación.
        // Más celdas alcanzables indican mayor flexibilidad.
        if (flexibilidad >= size) {
            return 30; // Alta flexibilidad.
        } else if (flexibilidad >= size / 2) {
            return 20; // Buena flexibilidad.
        } else if (flexibilidad >= size / 4) {
            return 10; // Flexibilidad moderada.
        } else {
            return -10; // Baja flexibilidad.
        }
    }

    /**
     * Realiza una búsqueda en anchura (BFS) para determinar las celdas de destino alcanzables
     * desde una fuente dada.
     *
     * @param gameState Estado actual del juego.
     * @param fuente    Punto de inicio de la búsqueda.
     * @param playerId  Identificador numérico del jugador.
     * @param destinos  Lista de puntos que representan los destinos finales.
     * @return Conjunto de puntos alcanzables desde la fuente hacia los destinos.
     */
    private Set<Point> bfsAlcanzables(HexGameStatus gameState, Point fuente, int playerId, List<Point> destinos) {
        Set<Point> alcanzables = new HashSet<>();
        Queue<Point> queue = new LinkedList<>();
        boolean[][] visitado = new boolean[gameState.getSize()][gameState.getSize()];

        queue.add(fuente);
        visitado[fuente.x][fuente.y] = true;

        // Definir movimientos posibles (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        while (!queue.isEmpty()) {
            Point actual = queue.poll();
            if (destinos.contains(actual)) {
                alcanzables.add(actual);
                continue; // No es necesario explorar más desde aquí.
            }

            for (Point d : directions) {
                int nx = actual.x + d.x;
                int ny = actual.y + d.y;
                Point vecino = new Point(nx, ny);
                if (estaDentro(vecino, gameState.getSize()) && !visitado[nx][ny]) {
                    int cell = gameState.getPos(nx, ny);
                    if (cell == playerId || cell == 0) { // Puede mover a celdas propias o vacías.
                        visitado[nx][ny] = true;
                        queue.add(vecino);
                    }
                }
            }
        }

        return alcanzables;
    }

    /**
     * Evalúa y cuenta los puentes virtuales para un jugador específico.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador para el cual se evalúan los puentes virtuales.
     * @return Número de puentes virtuales detectados.
     */
    private int evaluarPuentesVirtuales(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = obtenerFichasPropias(gameState, player);
        int size = gameState.getSize();
        int playerId = playerToId(player);
        int puentes = 0;

        // Definir direcciones hexagonales.
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };

        // Utilizar un conjunto para evitar contar el mismo puente múltiples veces.
        Set<String> puentesDetectados = new HashSet<>();

        for (Point ficha : fichas) {
            for (Point dir : directions) {
                int nx1 = ficha.x + dir.x;
                int ny1 = ficha.y + dir.y;
                int nx2 = ficha.x + 2 * dir.x;
                int ny2 = ficha.y + 2 * dir.y;

                Point intermedio = new Point(nx1, ny1);
                Point destino = new Point(nx2, ny2);

                if (estaDentro(intermedio, size) && estaDentro(destino, size)) {
                    // Verificar si el intermedio está vacío y el destino pertenece al mismo jugador.
                    if (gameState.getPos(intermedio.x, intermedio.y) == 0 &&
                        gameState.getPos(destino.x, destino.y) == playerId) {

                        // Crear una clave única para el puente para evitar duplicados.
                        String clavePuente = crearClavePuente(ficha, destino);
                        if (!puentesDetectados.contains(clavePuente)) {
                            puentesDetectados.add(clavePuente);
                            puentes++;
                        }
                    }
                }
            }
        }

        return puentes;
    }

    /**
     * Convierte PlayerType a ID numérico.
     *
     * @param player Tipo de jugador.
     * @return Identificador numérico del jugador.
     */
    private int playerToId(PlayerType player) {
        return (player == PlayerType.PLAYER1) ? 1 : -1;
    }

    /**
     * Recupera todas las fichas pertenecientes a un jugador específico.
     *
     * @param gameState Estado actual del juego.
     * @param player    Jugador cuyos fichas se obtienen.
     * @return Lista de puntos que representan las fichas del jugador.
     */
    private ArrayList<Point> obtenerFichasPropias(HexGameStatus gameState, PlayerType player) {
        ArrayList<Point> fichas = new ArrayList<>();
        int size = gameState.getSize();
        int playerId = playerToId(player);

        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == playerId) {
                    fichas.add(new Point(x, y));
                }
            }
        }
        return fichas;
    }

    /**
     * Verifica si una posición está dentro de los límites del tablero.
     *
     * @param p    Punto a verificar.
     * @param size Tamaño del tablero.
     * @return Verdadero si está dentro, falso en caso contrario.
     */
    private boolean estaDentro(Point p, int size) {
        return p.x >= 0 && p.y >= 0 && p.x < size && p.y < size;
    }

    /**
     * Calcula la distancia mínima hasta la victoria para un jugador dado utilizando
     * una versión optimizada del algoritmo de Dijkstra.
     *
     * @param gameState Estado actual del juego.
     * @param p         Jugador para el cual se calcula la distancia.
     * @return Distancia mínima en "pasos" necesarias. Si no hay camino, retorna un valor grande.
     */
    private int calcularDistanciaMinima(HexGameStatus gameState, PlayerType p) {
        int size = gameState.getSize();
        int playerId = (p == PlayerType.PLAYER1) ? 1 : -1;

        // Crear lista de nodos fuente según el jugador.
        List<Point> fuentes = (p == PlayerType.PLAYER1) ? getTransitableTop(gameState, playerId) : getTransitableLeft(gameState, playerId);

        // Si no hay fuentes transitables, retornar distancia muy grande.
        if (fuentes.isEmpty()) return 999999;

        // Inicializar matrices de distancia.
        int[][] dist = new int[size][size];
        for (int i = 0; i < size; i++) {
            Arrays.fill(dist[i], Integer.MAX_VALUE);
        }

        // PriorityQueue optimizada para Dijkstra.
        PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));

        // Inicializar distancias desde las fuentes.
        for (Point f : fuentes) {
            int cost = getCost(gameState, f.x, f.y, playerId);
            if (cost != Integer.MAX_VALUE) {
                dist[f.x][f.y] = cost;
                pq.add(new Node(f.x, f.y, cost));
            }
        }

        // Definir movimientos posibles en Hex.
        Point[] directions = {
            new Point(0, 1), new Point(1, 0), new Point(1, -1),
            new Point(0, -1), new Point(-1, 0), new Point(-1, 1)
        };

        // Ejecutar Dijkstra.
        while (!pq.isEmpty()) {
            Node current = pq.poll();

            // Ignorar nodos ya procesados con menor distancia.
            if (current.distance > dist[current.x][current.y]) continue;

            // Verificar si se ha llegado al borde contrario.
            if ((p == PlayerType.PLAYER1 && current.x == size - 1) ||
                (p == PlayerType.PLAYER2 && current.y == size - 1)) {
                return current.distance;
            }

            // Explorar vecinos.
            for (Point d : directions) {
                int nx = current.x + d.x;
                int ny = current.y + d.y;
                if (nx < 0 || ny < 0 || nx >= size || ny >= size) continue;

                int c = getCost(gameState, nx, ny, playerId);
                if (c == Integer.MAX_VALUE) continue; // No se puede pasar por el rival.

                int newDist = dist[current.x][current.y] + c;
                if (newDist < dist[nx][ny]) {
                    dist[nx][ny] = newDist;
                    pq.add(new Node(nx, ny, newDist));
                }
            }
        }

        // No se encontró conexión.
        return 999999;
    }

    /**
     * Devuelve el coste de pisar la celda (x,y) para el jugador playerId:
     * - 0 si la celda pertenece al playerId.
     * - 1 si la celda está vacía.
     * - Integer.MAX_VALUE si la celda pertenece al rival (infranqueable).
     *
     * @param gameState Estado actual del juego.
     * @param x         Coordenada X de la celda.
     * @param y         Coordenada Y de la celda.
     * @param playerId Identificador numérico del jugador.
     * @return Costo de la celda.
     */
    private int getCost(HexGameStatus gameState, int x, int y, int playerId) {
        int cell = gameState.getPos(x, y);
        if (cell == playerId) return 0;
        if (cell == 0) return 1;
        // cell == -playerId => rival
        return (cell == -playerId) ? Integer.MAX_VALUE : 1;
    }

    /**
     * Recupera las celdas transitables desde la parte superior del tablero para PLAYER1.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableTop(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(0, y) != -playerId) {
                fuentes.add(new Point(0, y));
            }
        }
        return fuentes;
    }

    /**
     * Recupera las celdas transitables desde la parte izquierda del tablero para PLAYER2.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableLeft(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, 0) != -playerId) {
                fuentes.add(new Point(x, 0));
            }
        }
        return fuentes;
    }

    /**
     * Recupera las celdas transitables desde la parte inferior del tablero para PLAYER1.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableBottom(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int y = 0; y < size; y++) {
            if (gameState.getPos(size - 1, y) != -playerId) {
                fuentes.add(new Point(size - 1, y));
            }
        }
        return fuentes;
    }

    /**
     * Recupera las celdas transitables desde la parte derecha del tablero para PLAYER2.
     *
     * @param gameState Estado actual del juego.
     * @param playerId Identificador numérico del jugador.
     * @return Lista de puntos transitables.
     */
    private List<Point> getTransitableRight(HexGameStatus gameState, int playerId) {
        List<Point> fuentes = new ArrayList<>();
        int size = gameState.getSize();
        for (int x = 0; x < size; x++) {
            if (gameState.getPos(x, size - 1) != -playerId) {
                fuentes.add(new Point(x, size - 1));
            }
        }
        return fuentes;
    }

    /**
     * Evalúa conexiones virtuales alrededor de una ficha específica para un jugador.
     * Considera patrones que pueden favorecer o perjudicar al jugador.
     *
     * @param gameState Estado actual del juego.
     * @param ficha     Ficha desde la cual se evalúan las conexiones.
     * @param player    Jugador para el cual se evalúan las conexiones.
     * @return Puntuación basada en las conexiones virtuales.
     */
    private int evaluarConexionesVirtuales(HexGameStatus gameState, Point ficha, PlayerType player) {
        int playerId = playerToId(player);
        // Definir direcciones únicas y relevantes (6 direcciones hexagonales).
        Point[] directions = {
            new Point(1, 0), new Point(0, 1), new Point(1, -1),
            new Point(-1, 1), new Point(-1, 0), new Point(0, -1)
        };
        int score = 0;

        for (Point d : directions) {
            Point neighbor = new Point(ficha.x + d.x, ficha.y + d.y);
            if (estaDentro(neighbor, gameState.getSize())) {
                if (gameState.getPos(neighbor.x, neighbor.y) == playerId) {
                    score += 20; // Conexión directa.
                } else if (gameState.getPos(neighbor.x, neighbor.y) == 0) {
                    // Verificar si hay una conexión potencial a través de celdas vacías.
                    score += 10;
                } else {
                    score -= 10; // Bloqueo potencial.
                }
            }
        }
        return score;
    }

    /**
     * Clase interna para implementar la estructura de datos Union-Find.
     * Utilizada para evaluar la conectividad de las fichas.
     */
    public static class UnionFind {
        int[] parent;

        /**
         * Inicializa la estructura de Union-Find.
         *
         * @param size Número total de elementos.
         */
        UnionFind(int size) {
            parent = new int[size];
            for (int i = 0; i < size; i++) parent[i] = i;
        }

        /**
         * Encuentra la raíz del elemento x con compresión de caminos.
         *
         * @param x Elemento a encontrar.
         * @return Raíz del elemento.
         */
        int find(int x) {
            if (parent[x] != x) parent[x] = find(parent[x]);
            return parent[x];
        }

        /**
         * Une los conjuntos que contienen a x e y.
         *
         * @param x Primer elemento.
         * @param y Segundo elemento.
         */
        void union(int x, int y) {
            parent[find(x)] = find(y);
        }
    }

    /**
     * Clase interna para representar nodos en el algoritmo de Dijkstra.
     */
    public static class Node implements Comparable<Node> {
        int x, y, distance;

        Node(int x, int y, int dist) {
            this.x = x;
            this.y = y;
            this.distance = dist;
        }

        @Override
        public int compareTo(Node other) {
            return Integer.compare(this.distance, other.distance);
        }
    }

}
