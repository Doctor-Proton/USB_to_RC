#include <stdint.h>
#define ATTR_PACKED                      __attribute__ ((packed))
#pragma once
typedef struct ATTR_PACKED 
{
    uint8_t Size; /**< Size of the descriptor, in bytes. */
	uint8_t Type; /**< Type of the descriptor, either a value in \ref USB_DescriptorTypes_t or a value
	             *   given by the specific class.
				               */
} USB_Descriptor_Header_t;

			#if !defined(__DOXYGEN__)
				// Obsolete, retained for compatibility with user code
				#define MACROS                  do
				#define MACROE                  while (0)
			#endif

			/** Convenience macro to determine the larger of two values.
			 *
			 *  \attention This macro should only be used with operands that do not have side effects from being evaluated
			 *             multiple times.
			 *
			 *  \param[in] x  First value to compare
			 *  \param[in] y  First value to compare
			 *
			 *  \return The larger of the two input parameters
			 */
			#if !defined(MAX) || defined(__DOXYGEN__)
				#define MAX(x, y)               (((x) > (y)) ? (x) : (y))
			#endif

			/** Convenience macro to determine the smaller of two values.
			 *
			 *  \attention This macro should only be used with operands that do not have side effects from being evaluated
			 *             multiple times.
			 *
			 *  \param[in] x  First value to compare.
			 *  \param[in] y  First value to compare.
			 *
			 *  \return The smaller of the two input parameters
			 */
			#if !defined(MIN) || defined(__DOXYGEN__)
				#define MIN(x, y)               (((x) < (y)) ? (x) : (y))
			#endif

			#if !defined(STRINGIFY) || defined(__DOXYGEN__)
				/** Converts the given input into a string, via the C Preprocessor. This macro puts literal quotation
				 *  marks around the input, converting the source into a string literal.
				 *
				 *  \param[in] x  Input to convert into a string literal.
				 *
				 *  \return String version of the input.
				 */
				#define STRINGIFY(x)            #x

				/** Converts the given input into a string after macro expansion, via the C Preprocessor. This macro puts
				 *  literal quotation marks around the expanded input, converting the source into a string literal.
				 *
				 *  \param[in] x  Input to expand and convert into a string literal.
				 *
				 *  \return String version of the expanded input.
				 */
				#define STRINGIFY_EXPANDED(x)   STRINGIFY(x)
			#endif

			#if !defined(CONCAT) || defined(__DOXYGEN__)
				/** Concatenates the given input into a single token, via the C Preprocessor.
				 *
				 *  \param[in] x  First item to concatenate.
				 *  \param[in] y  Second item to concatenate.
				 *
				 *  \return Concatenated version of the input.
				 */
				#define CONCAT(x, y)            x ## y

				/** CConcatenates the given input into a single token after macro expansion, via the C Preprocessor.
				 *
				 *  \param[in] x  First item to concatenate.
				 *  \param[in] y  Second item to concatenate.
				 *
				 *  \return Concatenated version of the expanded input.
				 */
				#define CONCAT_EXPANDED(x, y)   CONCAT(x, y)
			#endif