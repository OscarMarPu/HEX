package edu.upc.epsevg.prop.hex.players;

import edu.upc.epsevg.prop.hex.HexGameStatus;
import edu.upc.epsevg.prop.hex.PlayerType;
import java.awt.Point;

public class HexHeuristica {
    
    public int evaluarEstado(HexGameStatus gameState) {
        PlayerType currentPlayer = gameState.getCurrentPlayer();
        //boolean objetivoHorizontal = (currentPlayer == PlayerType.PLAYER1);

        // Identificar al rival y su objetivo
        PlayerType rival = (currentPlayer == PlayerType.PLAYER1) ? PlayerType.PLAYER2 : PlayerType.PLAYER1;
        //boolean rivalObjetivoHorizontal = (rival == PlayerType.PLAYER1);

        int size = gameState.getSize();
        int score = 0;

        int myId = (currentPlayer == PlayerType.PLAYER1) ? 1 : -1;
        int rivalId = (rival == PlayerType.PLAYER1) ? 1 : -1;
        
        /*for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                int cell = gameState.getPos(x, y);
                if (cell == myId) {
                    if (objetivoHorizontal) {
                        score += y;
                    } else {
                        score += x;
                    }
                }
            }
        }*/

        // Recorremos el tablero buscando fichas del rival.
        for (int x = 0; x < size; x++) {
            for (int y = 0; y < size; y++) {
                if (gameState.getPos(x, y) == rivalId) {
                    // Hemos encontrado una ficha del rival en (x, y).
                    // Intentaremos colocar una ficha en una diagonal concreta, por ejemplo la superior-izquierda (x-1, y-1).
                    Point diag1 = new Point(x-1, y-1);
                    Point diag2 = new Point(x+1, y+1); // Diagonal opuesta: inferior-derecha

                    // Verificamos si diag1 está dentro del tablero
                    if (isInsideBoard(diag1, size)) {
                        int cellDiag1 = gameState.getPos(diag1.x, diag1.y);
                        if (cellDiag1 == myId) {
                            // Ya tenemos una ficha nuestra en esa diagonal, excelente
                            score += 100;
                        } else if (cellDiag1 == 0) {
                            // La diagonal está vacía, sería un buen sitio para colocar nuestra ficha
                            score += 50;
                        } else {
                            // La diagonal no está libre para nosotros, probamos la otra diagonal
                            if (isInsideBoard(diag2, size)) {
                                int cellDiag2 = gameState.getPos(diag2.x, diag2.y);
                                if (cellDiag2 == myId) {
                                    score += 80;
                                } else if (cellDiag2 == 0) {
                                    score += 40;
                                }
                            }
                        }
                    } else {
                        // diag1 no está en el tablero, probamos diag2 directamente
                        if (isInsideBoard(diag2, size)) {
                            int cellDiag2 = gameState.getPos(diag2.x, diag2.y);
                            if (cellDiag2 == myId) {
                                score += 80;
                            } else if (cellDiag2 == 0) {
                                score += 40;
                            }
                        }
                    }
                }
            }
        }

        return score;
    }

    private boolean isInsideBoard(Point p, int size) {
        return p.x >= 0 && p.x < size && p.y >= 0 && p.y < size;
    }
}
