#ifndef CML_POOLALLOCATOR_H
#define CML_POOLALLOCATOR_H

namespace CML {

    // http://dmitrysoshnikov.com/compilers/writing-a-pool-allocator/
    /**
 * The allocator class.
 *
 * Features:
 *
 *   - Parametrized by number of chunks per block
 *   - Keeps track of the allocation pointer
 *   - Bump-allocates chunks
 *   - Requests a new larger block when needed
 *
 */
    class PoolAllocator {
        struct Chunk {
            /**
             * When a chunk is free, the `next` contains the
             * address of the next chunk in a list.
             *
             * When it's allocated, this space is used by
             * the user.
             */
            Chunk *next;
        };

    public:
        PoolAllocator(size_t chunksPerBlock)
                : mChunksPerBlock(chunksPerBlock) {
        }

        void *allocate(size_t size) {

            LockGuard lg(mMutex);

            // No chunks left in the current block, or no any block
            // exists yet. Allocate a new one, passing the chunk size:

            if (mAlloc == nullptr) {
                mAlloc = allocateBlock(size);
            }

            // The return value is the current position of
            // the allocation pointer:

            Chunk *freeChunk = mAlloc;

            // Advance (bump) the allocation pointer to the next chunk.
            //
            // When no chunks left, the `mAlloc` will be set to `nullptr`, and
            // this will cause allocation of a new block on the next request:

            mAlloc = mAlloc->next;

            return freeChunk;
        }

        void deallocate(void *chunk, size_t size = -1) {

            LockGuard lg(mMutex);

            // The freed chunk's next pointer points to the
            // current allocation pointer:

            reinterpret_cast<Chunk *>(chunk)->next = mAlloc;

            // And the allocation pointer is now set
            // to the returned (free) chunk:

            mAlloc = reinterpret_cast<Chunk *>(chunk);
        }

    private:
        Mutex mMutex;

        /**
         * Number of chunks per larger block.
         */
        size_t mChunksPerBlock;

        /**
         * Allocation pointer.
         */
        Chunk *mAlloc = nullptr;

        /**
         * Allocates a larger block (pool) for chunks.
         */
        Chunk *allocateBlock(size_t chunkSize) {

            size_t blockSize = mChunksPerBlock * chunkSize;

            // The first chunk of the new block.
            Chunk *blockBegin = reinterpret_cast<Chunk *>(Eigen::internal::handmade_aligned_malloc(blockSize));

            // Once the block is allocated, we need to chain all
            // the chunks in this block:

            Chunk *chunk = blockBegin;

            for (int i = 0; i < mChunksPerBlock - 1; ++i) {
                chunk->next =
                        reinterpret_cast<Chunk *>(reinterpret_cast<char *>(chunk) + chunkSize);
                chunk = chunk->next;
            }

            chunk->next = nullptr;

            return blockBegin;
        }
    };

}

#endif