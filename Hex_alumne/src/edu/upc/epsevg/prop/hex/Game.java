package edu.upc.epsevg.prop.hex;

import edu.upc.epsevg.prop.hex.players.H_E_X_Player;
import edu.upc.epsevg.prop.hex.players.HumanPlayer;
import edu.upc.epsevg.prop.hex.players.PlayerMinimax;
import edu.upc.epsevg.prop.hex.players.RandomPlayer;



import javax.swing.SwingUtilities;

/**
 * Checkers: el joc de taula.
 * @author bernat
 */
public class Game {
        /**
     * @param args
     */
    public static void main(String[] args) { 
        
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                
                IPlayer player2 = new H_E_X_Player(2/*GB*/);
                //IPlayer player2 = new RandomPlayer("random");
                PlayerMinimax player1 = new PlayerMinimax("Oscar i Sergi", 5, true);

                //IPlayer player1 = new HumanPlayer("Human");
                                
                new Board(player1 , player2, 11/*mida*/,  10/*s*/, false);
             }
        });
    }
}
