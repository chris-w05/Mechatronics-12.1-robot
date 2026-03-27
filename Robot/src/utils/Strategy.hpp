/**
 * @file Strategy.hpp
 * @brief Game-state singleton that tracks tool levels, block possession, and scoring logic.
 *
 * `Strategy` is a singleton (access via `Strategy::getInstance()`) that encodes
 * the current game progression:
 * - Which pickaxe/sword/shield level has been achieved.
 * - Which blocks are currently carried (Left, Hold, Right slots).
 * - Which scoring cycle (`CycleType`) the robot is working toward.
 * - Lookup tables for mining hit-counts and maximal minable block per tool level.
 *
 * Board index layout (for reference):
 * @code
 *           0   1   2
 * chest ->  3   4   5  <- start
 *           6   7   8
 *              mine
 * @endcode
 *
 * Usage:
 * @code
 * Strategy::getInstance().setPickaxeLevel(Strategy::STONE_TOOL);
 * auto maxBlock = Strategy::getInstance().getMaxMineableBlock();
 * @endcode
 */
//Strategy.hpp
//This allows the robot to know it's current state in the game
//It keeps track of information such as what what level pickaxe/ sword is owned
#ifndef STRATEGY_H
#define STRATEGY_H

/*Board indexes
            0   1   2
chest side  3   4   5   start side
            6   7   8
            mine side
*/

/* Whenever Strategy is called, use it in this style:

Strategy::getInstance().setPickaxeLevel(Strategy::STONE);
auto maxBlock = Strategy::getInstance().getMaxMineableBlock();

I went with a singleton style over a static class since it keeps the nature of Strategy as an object, but means I don't need to pass it between all my classes
*/

/*Overall strategy:
    until _pickaxeLevel = DIAMOND:
        place two wood blocks on cells 1 and 4
        acquire 3 of highest available material, reject if not highest available block 
        if silverfish detected, kill
        place in cells 6, 7, 8
    
    else:
        get 1 wood
        get 2 of same block, reject if not same as first
        place in cells 6, 7, 8

*/


/*How block allocation should be done
    flag: blockRecieved
    When a block's type is measured, whereToIndexBlock should be called once.
    Then, when the block is mined, the placement of the block in the bucket can be allocated using the information from whereToIndexBlock
    W
*/



/**
 * @brief Game-state manager and strategy decision engine.
 *
 * Tracks tool upgrade progression, block possession, and provides helper
 * methods to decide where to route a newly acquired block (`whereToIndexBlock`).
 */
class Strategy{

    public :
        /// @brief Scoring cycle type — determines which craft item is being assembled.
        enum CycleType
        {
            PICKAXE_HANDLE, ///< First cycle: place two wood blocks in wood slots
            PICKAXE_TIP,    ///< Second cycle: fill three slots with max-level blocks
            SWORD,          ///< Third cycle: one wood + two matching high-level blocks
            SHIELD          ///< Fourth cycle (future)
        };

        /// @brief Tool upgrade level, determines what blocks can be mined.
        enum ToolLevel
        {
            WOOD_TOOL    = 0, ///< Wood pickaxe (starting level)
            STONE_TOOL   = 1, ///< Stone pickaxe
            IRON_TOOL    = 2, ///< Iron pickaxe
            DIAMOND_TOOL = 3  ///< Diamond pickaxe (max level)
        };

        /// @brief Block material type.
        enum Block
        {
            NONE    = -1, ///< No block present
            WOOD    =  0, ///< Wood block
            STONE   =  1, ///< Stone block
            IRON    =  2, ///< Iron block
            DIAMOND =  3  ///< Diamond block
        };

        /// @brief Routing decision for a newly acquired block.
        enum IndexDirection
        {
            LEFT   =  0, ///< Move block to the left bucket slot
            REJECT = -1, ///< Eject block (not needed for current cycle)
            RIGHT  =  2, ///< Move block to the right bucket slot
            HOLD   =  1, ///< Move block to the centre (hold) slot
            CHEST  =  3  ///< Send block to the chest (rejected blocks scored passively)
        };

        /**
         * @brief Access the global Strategy singleton.
         * @return Reference to the single Strategy instance.
         */
        static Strategy &getInstance()
        {
            static Strategy instance; // Created once, on first call
            return instance;
        }

        /**
         * @brief Return the highest block type that can be mined with the current pickaxe.
         * @return Block enum value representing the best achievable block.
         */
        Block getMaxMineableBlock(){
            return _maxBlockMineable[_pickaxeLevel];
        }

        /**
         * @brief Return the number of hits needed to kill a silverfish with the current sword.
         * @return Hit count.
         */
        short hitsToKillSilverfish(){
            return _silverfishHits[_swordLevel];
        }

        /**
         * @brief Return the number of hits needed to mine a given block type.
         * @param blockToMine  The target block material.
         * @return             Hit count; -1 indicates the block cannot be mined at current level.
         */
        short hitsToMine(Block blockToMine){
            return _miningChart[blockToMine][_pickaxeLevel];
        }

        /** @brief Set the current pickaxe tool level. @param level New tool level. */
        void setPickaxeLevel(ToolLevel level){
            _pickaxeLevel = level;
        }

        /** @brief Set the current sword tool level. @param level New tool level. */
        void setSwordLevel( ToolLevel level){
            _swordLevel = level;
        }

        /**
         * @brief Decide where to route a newly acquired block.
         *
         * Evaluates the current cycle type and possession state to determine the
         * optimal slot for a newly collected block.
         * @param newBlock  The block type just collected.
         * @return          IndexDirection indicating where to send the block.
         */
        IndexDirection whereToIndexBlock(Block newBlock){
            //Fill wood, biasing left
            if( _nextItem == PICKAXE_HANDLE){
                if( newBlock == WOOD){
                    //If there is already a block there, HOLD, otherwise, push it left
                    if (_possession[LEFT] != NONE) return HOLD;
                    else return LEFT;
                }
                return REJECT;
            }

            //Left, Right, center order. 
            //Also needs to make sure all blocks are same type
            if (_nextItem == PICKAXE_TIP){
                if( newBlock != getMaxMineableBlock()){
                    //Potentially only chest valuable blocks - woudl require a more thourough analysis 
                    return CHEST;
                }
                //All blocks owned will be max level

                //Put block in LEFT if empty, same with RIGHT
                if( _possession[LEFT] == NONE) return LEFT;
                if (_possession[RIGHT] == NONE) return RIGHT;
                //HOLD if left and right are filled
                return IndexDirection::HOLD;
            }


            if (_nextItem == SWORD){
                //if new block is wood
                if (newBlock == WOOD){
                    //Fill to left if empty
                    if( _possession[LEFT] == NONE) return LEFT;
                    //otherwise get rid of the block
                    return CHEST;
                }

                //Blocks in this part are guaranteed to not be wood 
                //Take whatever possible - this could be changed later depending on block frequency
                if( _possession[RIGHT] == NONE) return RIGHT;
                if( newBlock != _possession[RIGHT]) return REJECT;
                return HOLD;
            }

            return REJECT;
        }


        /** @brief Clear all possession slots to NONE. */
        void clearAllPossession()
        {
            _possession[LEFT] = NONE;
            _possession[HOLD] = NONE; // Assuming HOLD=1 is center/temporary
            _possession[RIGHT] = NONE;
        }

        /**
         * @brief Return whether the robot has collected the full set of blocks needed to score.
         * @return `true` if all required slots are filled correctly for the current cycle.
         */
        bool readyToScore()
        {
            if (_nextItem == PICKAXE_HANDLE)
            {
                //Both wood in pos.
                return (_possession[LEFT] == WOOD && _possession[HOLD] == WOOD); 
            }
            else if (_nextItem == PICKAXE_TIP)
            {   
                //Is posession full of max mineable block
                Block tipType = getMaxMineableBlock();
                return (_possession[LEFT] == tipType && _possession[HOLD] == tipType && _possession[RIGHT] == tipType);
            }
            else if (_nextItem == SWORD)
            {
                //similar to pickaxe
                return (_possession[LEFT] == WOOD && _possession[RIGHT] != NONE && _possession[HOLD] == _possession[RIGHT]); // Wood + two matching
            }
            return false;
        }

        /** @brief Return the current scoring cycle target. */
        CycleType getNextItem() const { return _nextItem; }

        /**
         * @brief Set the block type held in a specific slot.
         * @param idx  Slot to update (LEFT, HOLD, or RIGHT). REJECT is ignored.
         * @param b    Block type to store.
         */
        void setPossession(IndexDirection idx, Block b)
        {
            if (idx == REJECT)
                return;
            if (idx >= LEFT && idx <= RIGHT)
                _possession[idx] = b;
        }

        /**
         * @brief Advance to the next scoring cycle and upgrade the pickaxe if applicable.
         *
         * Progression order: PICKAXE_HANDLE → PICKAXE_TIP (with tool upgrade) → SWORD.
         * Clears possession after advancing.
         */
        void advanceCycle()
        {
            if (_nextItem == PICKAXE_HANDLE)
            {
                _nextItem = PICKAXE_TIP;
            }
            else if (_nextItem == PICKAXE_TIP && _pickaxeLevel != DIAMOND_TOOL)
            {
                switch (_pickaxeLevel)
                {
                case WOOD_TOOL:
                    _pickaxeLevel = STONE_TOOL;
                    break;
                case STONE_TOOL:
                    _pickaxeLevel = IRON_TOOL;
                    break;
                case IRON_TOOL:
                    _pickaxeLevel = DIAMOND_TOOL;
                    break;
                default:
                    break;
                }
                _nextItem = PICKAXE_HANDLE;
            }
            else
            {
                _nextItem = SWORD;
            }
            clearAllPossession();
        }

        /** @brief Return the last block type that was detected by the sensor. */
        Block lastSeenBlock(){ return _lastSeenBlock;};

    private:
        ToolLevel _pickaxeLevel = ToolLevel::WOOD_TOOL; ///< Current pickaxe upgrade level
        ToolLevel _swordLevel   = ToolLevel::WOOD_TOOL; ///< Current sword upgrade level
        CycleType _nextItem     = CycleType::PICKAXE_HANDLE; ///< Scoring cycle currently being assembled

        Block _possession[3] = {Block::NONE, Block::NONE, Block::NONE}; ///< Held blocks: [LEFT, HOLD, RIGHT]

        bool _readyToPlace = false; ///< Flag indicating all slots are filled and ready to score

        Block _lastSeenBlock = Block::NONE; ///< Most recently sensed block type

        /// Mining hit-count lookup table: `_miningChart[blockType][toolLevel]`. -1 = unmineble.
        const short _miningChart[4][4] =
            {
                {5, 4, 3, 1},
                {10, 5, 3, 2},
                {-1, 10, 5, 3},
                {-1, -1, 10, 5},
        };

        const short _silverfishHits[4] = {10, 7, 4, 1}; ///< Silverfish kill-hit counts per sword level

        /// Highest block type minable per pickaxe upgrade level.
        const Block _maxBlockMineable[4] = 
            {Block::STONE, Block::IRON, Block::DIAMOND, Block::DIAMOND};
        


};

#endif